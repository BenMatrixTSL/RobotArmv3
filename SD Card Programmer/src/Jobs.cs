using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Compression;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using Microsoft.Win32.SafeHandles;

namespace SdCardProgrammer
{
    public enum JobMode { Image, Zip }

    public class JobOptions
    {
        public JobMode Mode;
        public string SourcePath;
        public int DiskNumber;
        public bool Verify;
        public bool SkipUnpartitioned;
        public bool Format;
        public string FileSystem;
        public string Label;
        public bool StripRoot;
        public bool Eject;
    }

    public class JobContext
    {
        public volatile bool CancelRequested;
        public Action<string> Log;
        /// <summary>phase, bytes done, bytes total (-1 = unknown), fraction 0..1 (-1 = use bytes)</summary>
        public Action<string, long, long, double> Progress;

        public void CheckCancel()
        {
            if (CancelRequested) throw new OperationCanceledException();
        }
    }

    public class JobResult
    {
        public bool Ejected;
        public string Summary;
    }

    /// <summary>A readable image stream plus how to measure progress through it.</summary>
    class ImageSource : IDisposable
    {
        public Stream Stream;
        public long Length = -1;                 // uncompressed length if known
        public Func<double> Fraction;            // progress hint when Length is unknown
        public string Description;
        readonly List<IDisposable> owned = new List<IDisposable>();
        Process process;

        public static ImageSource Open(string path)
        {
            var src = new ImageSource();
            string lower = path.ToLowerInvariant();
            if (lower.EndsWith(".gz"))
            {
                var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20);
                src.owned.Add(fs);
                src.Stream = new GZipStream(fs, CompressionMode.Decompress);
                src.Fraction = () => fs.Length == 0 ? 0 : (double)fs.Position / fs.Length;
                src.Description = "gzip-compressed image";
            }
            else if (lower.EndsWith(".zip"))
            {
                ZipArchive zip = ZipFile.OpenRead(path);
                src.owned.Add(zip);
                ZipArchiveEntry best = null;
                foreach (var e in zip.Entries)
                {
                    string n = e.Name.ToLowerInvariant();
                    if ((n.EndsWith(".img") || n.EndsWith(".bin") || n.EndsWith(".iso")) && (best == null || e.Length > best.Length))
                        best = e;
                }
                if (best == null) throw new IOException("The ZIP file does not contain a .img file. (To copy a ZIP's files onto a card, use 'Copy ZIP contents' mode.)");
                src.Stream = best.Open();
                src.Length = best.Length;
                src.Description = "image '" + best.FullName + "' inside ZIP";
            }
            else if (lower.EndsWith(".xz") || lower.EndsWith(".7z"))
            {
                string sevenZip = FindSevenZip();
                if (sevenZip == null)
                    throw new IOException("Compressed .xz/.7z images need 7-Zip (https://www.7-zip.org) installed, or extract the .img first.");
                var psi = new ProcessStartInfo(sevenZip, "e -so -bd \"" + path + "\"");
                psi.UseShellExecute = false;
                psi.CreateNoWindow = true;
                psi.RedirectStandardOutput = true;
                psi.RedirectStandardError = true;
                src.process = Process.Start(psi);
                src.process.ErrorDataReceived += delegate { };
                src.process.BeginErrorReadLine();
                src.Stream = src.process.StandardOutput.BaseStream;
                src.Description = "compressed image (decompressing with 7-Zip)";
            }
            else
            {
                var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20, FileOptions.SequentialScan);
                src.Stream = fs;
                src.Length = fs.Length;
                src.Description = "raw image";
            }
            return src;
        }

        public static string FindSevenZip()
        {
            string[] candidates = {
                Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "tools\\7z.exe"),
                Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles), "7-Zip\\7z.exe"),
                @"C:\Program Files\7-Zip\7z.exe",
                @"C:\Program Files (x86)\7-Zip\7z.exe",
            };
            foreach (string c in candidates) if (File.Exists(c)) return c;
            return null;
        }

        public void Dispose()
        {
            try { if (Stream != null) Stream.Dispose(); } catch { }
            if (process != null)
            {
                try { if (!process.HasExited) process.Kill(); } catch { }
                process.Dispose();
            }
            foreach (var d in owned) { try { d.Dispose(); } catch { } }
        }
    }

    public static class Jobs
    {
        const int Chunk = 4 << 20;           // 4 MiB per write
        const int WipeBytes = 1 << 20;       // zero the first/last 1 MiB of the card before writing

        public static JobResult Run(JobOptions o, JobContext ctx)
        {
            DiskInfo disk = DiskTools.QueryDisk(o.DiskNumber);
            string reason;
            if (!DiskTools.IsSafeTarget(disk, DiskTools.SystemDiskNumber(), out reason)) throw new IOException(reason);
            if (!disk.HasMedia) throw new IOException("No card detected in the reader.");
            ctx.Log("Target: Disk " + disk.Number + " - " + disk.Model + ", " + DiskTools.FormatSize(disk.Size));

            JobResult result = o.Mode == JobMode.Image ? WriteImage(o, disk, ctx) : CopyZip(o, disk, ctx);

            if (o.Eject)
            {
                ctx.Progress("Ejecting card...", 0, -1, -1);
                result.Ejected = DiskTools.SafeEject(disk.Number, ctx.Log);
                ctx.Log(result.Ejected ? "Card ejected." : "Card dismounted (this reader does not support software eject) - safe to remove.");
            }
            else if (o.Mode == JobMode.Image)
            {
                DiskTools.UpdateDiskProperties(disk.Number);
            }
            return result;
        }

        // ------------------------------------------------------------------ image mode

        static JobResult WriteImage(JobOptions o, DiskInfo disk, JobContext ctx)
        {
            ctx.Progress("Opening image...", 0, -1, -1);
            using (ImageSource src = ImageSource.Open(o.SourcePath))
            {
                ctx.Log("Source: " + src.Description + (src.Length > 0 ? ", " + DiskTools.FormatSize(src.Length) : ""));
                byte[] buf = new byte[Chunk];
                int len = ReadFully(src.Stream, buf, Chunk);
                if (len == 0) throw new IOException("The image is empty.");

                // Work out how much to write - before touching the card.
                long limit = src.Length;
                if (o.SkipUnpartitioned)
                {
                    long partEnd = Mbr.LastPartitionEnd(buf, len);
                    if (partEnd > 0 && (limit < 0 || partEnd < limit))
                    {
                        if (limit > 0)
                            ctx.Log("Partitions end at " + DiskTools.FormatSize(partEnd) + "; skipping " +
                                    DiskTools.FormatSize(limit - partEnd) + " of unused space at the end of the image.");
                        limit = partEnd;
                    }
                }
                if (limit > disk.Size)
                    throw new IOException("The image (" + DiskTools.FormatSize(limit) + ") is larger than this card (" +
                                          DiskTools.FormatSize(disk.Size) + "). Use a bigger card or a shrunk image.");

                ctx.CheckCancel();
                ctx.Progress("Unmounting card...", 0, -1, -1);
                List<SafeFileHandle> locks = DiskTools.LockAndDismountVolumes(disk.Number, ctx.Log);
                IntPtr abuf = NativeMethods.VirtualAlloc(IntPtr.Zero, (UIntPtr)(uint)Chunk,
                    NativeMethods.MEM_COMMIT | NativeMethods.MEM_RESERVE, NativeMethods.PAGE_READWRITE);
                if (abuf == IntPtr.Zero) throw new OutOfMemoryException();
                try
                {
                    using (SafeFileHandle h = DiskTools.OpenOrThrow(DiskTools.PhysicalDrivePath(disk.Number),
                        NativeMethods.GENERIC_READ | NativeMethods.GENERIC_WRITE,
                        NativeMethods.FILE_FLAG_NO_BUFFERING | NativeMethods.FILE_FLAG_WRITE_THROUGH))
                    {
                        int sector = DiskTools.GetSectorSize(h);
                        long diskLen = DiskTools.GetLength(h);
                        if (diskLen <= 0) diskLen = disk.Size;

                        // Wipe old partition tables (MBR at the start, backup GPT at the end).
                        ZeroMemory(abuf, WipeBytes);
                        WriteAt(h, abuf, WipeBytes, 0);
                        long tail = (diskLen - WipeBytes) / sector * sector;
                        if (tail > WipeBytes) WriteAt(h, abuf, WipeBytes, tail);

                        MD5 md5 = o.Verify ? MD5.Create() : null;
                        byte[] sector0 = null;
                        long offset = 0, logical = 0;
                        ctx.Log("Writing image...");
                        while (len > 0)
                        {
                            ctx.CheckCancel();
                            if (limit > 0 && logical + len > limit) len = (int)(limit - logical);
                            if (len <= 0) break;
                            int padded = (len + sector - 1) / sector * sector;
                            if (padded > len) Array.Clear(buf, len, padded - len);
                            if (offset + padded > diskLen)
                                throw new IOException("The image is larger than this card (" + DiskTools.FormatSize(diskLen) + ").");
                            if (md5 != null) md5.TransformBlock(buf, 0, padded, null, 0);
                            Marshal.Copy(buf, 0, abuf, padded);

                            if (offset == 0)
                            {
                                // Hold back sector 0 (the partition table) until everything else is written,
                                // so an interrupted write never leaves a half-valid card.
                                sector0 = new byte[sector];
                                Array.Copy(buf, sector0, sector);
                                if (padded > sector) WriteAt(h, new IntPtr(abuf.ToInt64() + sector), padded - sector, sector);
                            }
                            else
                            {
                                WriteAt(h, abuf, padded, offset);
                            }
                            offset += padded;
                            logical += len;
                            ctx.Progress("Writing", logical, limit, limit > 0 ? -1 : (src.Fraction != null ? src.Fraction() : -1));

                            if (len < Chunk || (limit > 0 && logical >= limit)) break;
                            len = ReadFully(src.Stream, buf, Chunk);
                        }

                        Marshal.Copy(sector0, 0, abuf, sector);
                        WriteAt(h, abuf, sector, 0);
                        NativeMethods.FlushFileBuffers(h);
                        ctx.Log("Wrote " + DiskTools.FormatSize(logical) + ".");

                        if (md5 != null)
                        {
                            md5.TransformFinalBlock(new byte[0], 0, 0);
                            byte[] expected = md5.Hash;
                            ctx.Log("Verifying...");
                            MD5 check = MD5.Create();
                            long pos = 0;
                            while (pos < offset)
                            {
                                ctx.CheckCancel();
                                int n = (int)Math.Min(Chunk, offset - pos);
                                ReadAt(h, abuf, n, pos);
                                Marshal.Copy(abuf, buf, 0, n);
                                check.TransformBlock(buf, 0, n, null, 0);
                                pos += n;
                                ctx.Progress("Verifying", pos, offset, -1);
                            }
                            check.TransformFinalBlock(new byte[0], 0, 0);
                            if (Convert.ToBase64String(check.Hash) != Convert.ToBase64String(expected))
                                throw new IOException("VERIFY FAILED - the data read back from the card does not match the image. The card may be faulty or fake.");
                            ctx.Log("Verify OK (MD5 " + BitConverter.ToString(expected).Replace("-", "").ToLowerInvariant() + ").");
                        }
                        return new JobResult { Summary = "Image written" + (o.Verify ? " and verified" : "") + " (" + DiskTools.FormatSize(logical) + ")" };
                    }
                }
                finally
                {
                    NativeMethods.VirtualFree(abuf, UIntPtr.Zero, NativeMethods.MEM_RELEASE);
                    foreach (var l in locks) l.Dispose();
                }
            }
        }

        static void ZeroMemory(IntPtr p, int len)
        {
            var zeros = new byte[len];
            Marshal.Copy(zeros, 0, p, len);
        }

        static void WriteAt(SafeFileHandle h, IntPtr buf, int len, long offset)
        {
            long newPos;
            if (!NativeMethods.SetFilePointerEx(h, offset, out newPos, 0))
                throw new IOException("Seek failed at " + offset + ": " + LastError());
            int written;
            if (!NativeMethods.WriteFile(h, buf, len, out written, IntPtr.Zero) || written != len)
                throw new IOException("Write failed at " + DiskTools.FormatSize(offset) + ": " + LastError() +
                                      " (card removed, write-protected, or faulty?)");
        }

        static void ReadAt(SafeFileHandle h, IntPtr buf, int len, long offset)
        {
            long newPos;
            if (!NativeMethods.SetFilePointerEx(h, offset, out newPos, 0))
                throw new IOException("Seek failed at " + offset + ": " + LastError());
            int read;
            if (!NativeMethods.ReadFile(h, buf, len, out read, IntPtr.Zero) || read != len)
                throw new IOException("Read-back failed at " + DiskTools.FormatSize(offset) + ": " + LastError());
        }

        static string LastError()
        {
            return new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error()).Message;
        }

        static int ReadFully(Stream s, byte[] buf, int count)
        {
            int total = 0;
            while (total < count)
            {
                int n = s.Read(buf, total, count - total);
                if (n <= 0) break;
                total += n;
            }
            return total;
        }

        // ------------------------------------------------------------------ zip mode

        static JobResult CopyZip(JobOptions o, DiskInfo disk, JobContext ctx)
        {
            using (ZipArchive zip = ZipFile.OpenRead(o.SourcePath))
            {
                // Plan the copy first so problems show up before the card is wiped.
                string strip = o.StripRoot ? CommonRoot(zip) : null;
                if (strip != null) ctx.Log("Removing top-level folder '" + strip + "' from paths.");
                long total = 0, biggest = 0;
                int files = 0;
                foreach (var e in zip.Entries)
                {
                    if (IsDirectory(e)) continue;
                    total += e.Length;
                    files++;
                    biggest = Math.Max(biggest, e.Length);
                }
                ctx.Log("ZIP contains " + files + " files, " + DiskTools.FormatSize(total) + ".");
                bool fat32 = string.Equals(o.FileSystem, "FAT32", StringComparison.OrdinalIgnoreCase);
                if (o.Format && fat32 && biggest >= 4L * 1024 * 1024 * 1024)
                    throw new IOException("The ZIP contains a file of 4 GB or more, which FAT32 cannot store. Choose exFAT or NTFS.");

                char letter;
                if (o.Format)
                {
                    ctx.CheckCancel();
                    ctx.Progress("Formatting card (" + o.FileSystem + ")...", 0, -1, -1);
                    char preferred = disk.Letters.Count > 0 ? disk.Letters[0] : '\0';
                    if (preferred == '\0')
                    {
                        foreach (var kv in DiskTools.LetterMap()) if (kv.Value == disk.Number) { preferred = kv.Key; break; }
                    }
                    ctx.Log("Formatting card as " + o.FileSystem + ", label '" + o.Label + "'...");
                    letter = DiskTools.FormatWithDiskpart(disk.Number, disk.Size, o.FileSystem, o.Label, preferred, ctx.Log,
                        () => ctx.CancelRequested);
                    ctx.Log("Formatted; card mounted as " + letter + ":");
                }
                else
                {
                    letter = '\0';
                    foreach (var kv in DiskTools.LetterMap())
                        if (kv.Value == disk.Number) { letter = kv.Key; break; }
                    if (letter == '\0') throw new IOException("The card has no drive letter (it may be unformatted). Tick 'Format card first'.");
                }

                string root = letter + ":\\";
                var drive = new DriveInfo(root);
                if (drive.AvailableFreeSpace < total)
                    throw new IOException("Not enough space on the card: need " + DiskTools.FormatSize(total) +
                                          ", free " + DiskTools.FormatSize(drive.AvailableFreeSpace) + ".");

                ctx.Log("Copying files to " + root + " ...");
                string rootFull = Path.GetFullPath(root);
                byte[] buf = new byte[1 << 20];
                long done = 0;
                int copied = 0;
                foreach (var e in zip.Entries)
                {
                    ctx.CheckCancel();
                    string rel = e.FullName.Replace('/', '\\');
                    if (strip != null)
                    {
                        if (!rel.StartsWith(strip + "\\", StringComparison.OrdinalIgnoreCase)) continue;
                        rel = rel.Substring(strip.Length + 1);
                    }
                    rel = rel.TrimStart('\\');
                    if (rel.Length == 0) continue;
                    string dest = Path.GetFullPath(Path.Combine(root, rel));
                    if (!dest.StartsWith(rootFull, StringComparison.OrdinalIgnoreCase))
                        throw new IOException("Unsafe path in ZIP: " + e.FullName);

                    if (IsDirectory(e))
                    {
                        Directory.CreateDirectory(dest);
                        continue;
                    }
                    Directory.CreateDirectory(Path.GetDirectoryName(dest));
                    using (Stream input = e.Open())
                    using (var output = new FileStream(dest, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20))
                    {
                        int n;
                        while ((n = input.Read(buf, 0, buf.Length)) > 0)
                        {
                            ctx.CheckCancel();
                            output.Write(buf, 0, n);
                            done += n;
                            ctx.Progress("Copying", done, total, -1);
                        }
                    }
                    try { File.SetLastWriteTime(dest, e.LastWriteTime.DateTime); } catch { }
                    if (new FileInfo(dest).Length != e.Length)
                        throw new IOException("Size mismatch after copying " + rel);
                    copied++;
                }

                // Make sure everything is physically on the card.
                using (var vh = DiskTools.Open(@"\\.\" + letter + ":", NativeMethods.GENERIC_READ | NativeMethods.GENERIC_WRITE, 0))
                {
                    if (!vh.IsInvalid) NativeMethods.FlushFileBuffers(vh);
                }
                ctx.Log("Copied " + copied + " files (" + DiskTools.FormatSize(done) + ").");
                return new JobResult { Summary = copied + " files copied (" + DiskTools.FormatSize(done) + ")" };
            }
        }

        static bool IsDirectory(ZipArchiveEntry e)
        {
            return e.FullName.EndsWith("/") || e.FullName.EndsWith("\\");
        }

        /// <summary>If every entry sits under one top-level folder, returns that folder name.</summary>
        static string CommonRoot(ZipArchive zip)
        {
            string root = null;
            foreach (var e in zip.Entries)
            {
                string p = e.FullName.Replace('\\', '/').TrimStart('/');
                int slash = p.IndexOf('/');
                if (slash <= 0) return null;          // a file at the top level
                string first = p.Substring(0, slash);
                if (root == null) root = first;
                else if (!string.Equals(root, first, StringComparison.OrdinalIgnoreCase)) return null;
            }
            return root;
        }
    }

    public static class Mbr
    {
        /// <summary>
        /// End (in bytes) of the last partition in an MBR partition table, or -1 if the data
        /// isn't a plain MBR (GPT, superfloppy, unknown) - in which case the whole image is written.
        /// </summary>
        public static long LastPartitionEnd(byte[] s, int len)
        {
            if (len < 512 || s[510] != 0x55 || s[511] != 0xAA) return -1;
            // A FAT boot sector without a partition table also ends in 55 AA.
            if (Ascii(s, 54, 3) == "FAT" || Ascii(s, 82, 5) == "FAT32" || Ascii(s, 3, 4) == "NTFS" || Ascii(s, 3, 5) == "EXFAT")
                return -1;
            long end = -1;
            for (int i = 0; i < 4; i++)
            {
                int off = 446 + 16 * i;
                byte status = s[off];
                byte type = s[off + 4];
                if (status != 0x00 && status != 0x80) return -1;
                if (type == 0) continue;
                if (type == 0xEE) return -1;      // protective MBR -> GPT disk, keep everything
                long lba = BitConverter.ToUInt32(s, off + 8);
                long count = BitConverter.ToUInt32(s, off + 12);
                if (count == 0) continue;
                end = Math.Max(end, (lba + count) * 512);
            }
            return end;
        }

        static string Ascii(byte[] s, int off, int n)
        {
            return System.Text.Encoding.ASCII.GetString(s, off, n);
        }
    }
}
