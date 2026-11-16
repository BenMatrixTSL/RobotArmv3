using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Microsoft.Win32.SafeHandles;

namespace SdCardProgrammer
{
    /// <summary>A physical disk (card reader slot) as seen by Windows.</summary>
    public class DiskInfo
    {
        public int Number;
        public long Size;            // 0 when no card is inserted
        public bool HasMedia;
        public bool RemovableMedia;
        public int BusType;
        public string Vendor = "";
        public string Product = "";
        public List<char> Letters = new List<char>();

        public string BusName
        {
            get
            {
                switch (BusType)
                {
                    case 7: return "USB";
                    case 12: return "SD";
                    case 13: return "MMC";
                    default: return "bus " + BusType;
                }
            }
        }

        public string Model
        {
            get
            {
                string s = (Vendor + " " + Product).Trim();
                return s.Length == 0 ? "Unknown device" : s;
            }
        }

        public string LettersText
        {
            get
            {
                if (Letters.Count == 0) return "(no letter)";
                var parts = new List<string>();
                foreach (char c in Letters) parts.Add(c + ":");
                return string.Join(" ", parts.ToArray());
            }
        }

        public string Describe()
        {
            string media = HasMedia ? DiskTools.FormatSize(Size) : "no card inserted";
            return LettersText + "   " + media + "   " + Model + " (" + BusName + ")   [Disk " + Number + "]";
        }
    }

    public static class DiskTools
    {
        public const long LargeDiskWarningBytes = 256L * 1000 * 1000 * 1000;

        public static string FormatSize(long bytes)
        {
            if (bytes >= 1000L * 1000 * 1000) return (bytes / 1e9).ToString("0.0") + " GB";
            if (bytes >= 1000L * 1000) return (bytes / 1e6).ToString("0.0") + " MB";
            return (bytes / 1e3).ToString("0.0") + " KB";
        }

        public static string PhysicalDrivePath(int n) { return @"\\.\PhysicalDrive" + n; }

        public static SafeFileHandle Open(string path, uint access, uint flags)
        {
            return NativeMethods.CreateFile(path, access,
                NativeMethods.FILE_SHARE_READ | NativeMethods.FILE_SHARE_WRITE,
                IntPtr.Zero, NativeMethods.OPEN_EXISTING, flags, IntPtr.Zero);
        }

        public static SafeFileHandle OpenOrThrow(string path, uint access, uint flags)
        {
            var h = Open(path, access, flags);
            if (h.IsInvalid)
            {
                int err = Marshal.GetLastWin32Error();
                throw new IOException("Cannot open " + path + ": " + new Win32Exception(err).Message);
            }
            return h;
        }

        static bool Ioctl(SafeFileHandle h, uint code, byte[] input, byte[] output, out int returned)
        {
            return NativeMethods.DeviceIoControl(h, code, input, input == null ? 0 : input.Length,
                output, output == null ? 0 : output.Length, out returned, IntPtr.Zero);
        }

        static bool Ioctl(SafeFileHandle h, uint code)
        {
            int r;
            return Ioctl(h, code, null, null, out r);
        }

        /// <summary>Disk number that a drive letter lives on, or -1.</summary>
        public static int DiskNumberForPath(string devicePath)
        {
            using (var h = Open(devicePath, 0, 0))
            {
                if (h.IsInvalid) return -1;
                var outBuf = new byte[12];
                int r;
                if (!Ioctl(h, NativeMethods.IOCTL_STORAGE_GET_DEVICE_NUMBER, null, outBuf, out r)) return -1;
                uint deviceType = BitConverter.ToUInt32(outBuf, 0);
                if (deviceType != NativeMethods.FILE_DEVICE_DISK) return -1;
                return (int)BitConverter.ToUInt32(outBuf, 4);
            }
        }

        public static int DiskNumberForLetter(char letter)
        {
            return DiskNumberForPath(@"\\.\" + char.ToUpperInvariant(letter) + ":");
        }

        public static int SystemDiskNumber()
        {
            string win = Environment.GetFolderPath(Environment.SpecialFolder.Windows);
            if (string.IsNullOrEmpty(win)) win = @"C:\";
            return DiskNumberForLetter(win[0]);
        }

        /// <summary>Drive letter -> disk number for every lettered disk volume.</summary>
        public static Dictionary<char, int> LetterMap()
        {
            var map = new Dictionary<char, int>();
            uint mask = NativeMethods.GetLogicalDrives();
            for (int i = 0; i < 26; i++)
            {
                if ((mask & (1u << i)) == 0) continue;
                char c = (char)('A' + i);
                int n = DiskNumberForLetter(c);
                if (n >= 0) map[c] = n;
            }
            return map;
        }

        /// <summary>Query one physical disk. Returns null if it doesn't exist.</summary>
        public static DiskInfo QueryDisk(int n)
        {
            var info = new DiskInfo { Number = n };
            using (var h = Open(PhysicalDrivePath(n), 0, 0))
            {
                if (h.IsInvalid) return null;
                // STORAGE_PROPERTY_QUERY { StorageDeviceProperty, PropertyStandardQuery }
                var query = new byte[12];
                var desc = new byte[1024];
                int r;
                if (Ioctl(h, NativeMethods.IOCTL_STORAGE_QUERY_PROPERTY, query, desc, out r) && r >= 32)
                {
                    info.RemovableMedia = desc[10] != 0;
                    info.Vendor = ReadAnsi(desc, BitConverter.ToInt32(desc, 12), r);
                    info.Product = ReadAnsi(desc, BitConverter.ToInt32(desc, 16), r);
                    info.BusType = BitConverter.ToInt32(desc, 28);
                }
                info.HasMedia = Ioctl(h, NativeMethods.IOCTL_STORAGE_CHECK_VERIFY2);
            }
            if (info.HasMedia)
            {
                using (var h = Open(PhysicalDrivePath(n), NativeMethods.GENERIC_READ, 0))
                {
                    if (!h.IsInvalid) info.Size = GetLength(h);
                }
                if (info.Size <= 0) info.HasMedia = false;
            }
            return info;
        }

        static string ReadAnsi(byte[] buf, int offset, int limit)
        {
            if (offset <= 0 || offset >= limit) return "";
            int end = offset;
            while (end < limit && buf[end] != 0) end++;
            return Encoding.ASCII.GetString(buf, offset, end - offset).Trim();
        }

        public static long GetLength(SafeFileHandle h)
        {
            var outBuf = new byte[8];
            int r;
            if (!Ioctl(h, NativeMethods.IOCTL_DISK_GET_LENGTH_INFO, null, outBuf, out r)) return 0;
            return BitConverter.ToInt64(outBuf, 0);
        }

        public static int GetSectorSize(SafeFileHandle h)
        {
            var outBuf = new byte[256];
            int r;
            if (Ioctl(h, NativeMethods.IOCTL_DISK_GET_DRIVE_GEOMETRY_EX, null, outBuf, out r) && r >= 24)
            {
                int bps = BitConverter.ToInt32(outBuf, 20);
                if (bps >= 512 && bps <= 4096 && (bps & (bps - 1)) == 0) return bps;
            }
            return 512;
        }

        public static bool MediaPresent(int n)
        {
            using (var h = Open(PhysicalDrivePath(n), 0, 0))
            {
                if (h.IsInvalid) return false;
                return Ioctl(h, NativeMethods.IOCTL_STORAGE_CHECK_VERIFY2);
            }
        }

        /// <summary>
        /// Only USB / SD / MMC / removable-media disks that do NOT hold Windows are ever offered or written.
        /// </summary>
        public static bool IsSafeTarget(DiskInfo d, int systemDisk, out string reason)
        {
            reason = null;
            if (d == null) { reason = "Disk not found (was the card reader unplugged?)."; return false; }
            if (d.Number == systemDisk) { reason = "Disk " + d.Number + " holds Windows - refusing."; return false; }
            bool busOk = d.BusType == 7 || d.BusType == 12 || d.BusType == 13;
            if (!busOk && !d.RemovableMedia)
            {
                reason = "Disk " + d.Number + " is not a USB/SD card device (" + d.BusName + ") - refusing.";
                return false;
            }
            return true;
        }

        public static List<DiskInfo> ListCandidateDisks()
        {
            var result = new List<DiskInfo>();
            int systemDisk = SystemDiskNumber();
            var letters = LetterMap();
            for (int n = 0; n < 64; n++)
            {
                DiskInfo d = QueryDisk(n);
                if (d == null) continue;
                string reason;
                if (!IsSafeTarget(d, systemDisk, out reason)) continue;
                foreach (var kv in letters) if (kv.Value == n) d.Letters.Add(kv.Key);
                d.Letters.Sort();
                result.Add(d);
            }
            return result;
        }

        /// <summary>All volume GUID paths (no trailing slash) that have an extent on disk n.</summary>
        public static List<string> VolumesOnDisk(int n)
        {
            var list = new List<string>();
            var sb = new StringBuilder(260);
            IntPtr find = NativeMethods.FindFirstVolume(sb, sb.Capacity);
            if (find == NativeMethods.INVALID_HANDLE_VALUE) return list;
            try
            {
                do
                {
                    string vol = sb.ToString().TrimEnd('\\');
                    using (var h = Open(vol, 0, 0))
                    {
                        if (!h.IsInvalid)
                        {
                            var outBuf = new byte[8 + 24 * 32];
                            int r;
                            if (Ioctl(h, NativeMethods.IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS, null, outBuf, out r) && r >= 8)
                            {
                                int count = BitConverter.ToInt32(outBuf, 0);
                                for (int i = 0; i < count && 8 + 24 * i + 4 <= r; i++)
                                {
                                    if (BitConverter.ToInt32(outBuf, 8 + 24 * i) == n) { list.Add(vol); break; }
                                }
                            }
                        }
                    }
                    sb.Length = 0;
                } while (NativeMethods.FindNextVolume(find, sb, sb.Capacity));
            }
            finally { NativeMethods.FindVolumeClose(find); }
            return list;
        }

        /// <summary>
        /// Flush, lock and dismount every volume on disk n. The returned handles keep the locks held;
        /// dispose them when the raw disk work is finished.
        /// </summary>
        public static List<SafeFileHandle> LockAndDismountVolumes(int n, Action<string> log)
        {
            var handles = new List<SafeFileHandle>();
            foreach (string vol in VolumesOnDisk(n))
            {
                var h = Open(vol, NativeMethods.GENERIC_READ | NativeMethods.GENERIC_WRITE, 0);
                if (h.IsInvalid) { log("  Could not open volume " + vol + " (skipped)"); continue; }
                NativeMethods.FlushFileBuffers(h);
                bool locked = false;
                for (int i = 0; i < 30 && !locked; i++)
                {
                    locked = Ioctl(h, NativeMethods.FSCTL_LOCK_VOLUME);
                    if (!locked) Thread.Sleep(100);
                }
                if (!locked) log("  Warning: volume is in use by another program; forcing dismount.");
                Ioctl(h, NativeMethods.FSCTL_DISMOUNT_VOLUME);
                handles.Add(h);
            }
            return handles;
        }

        public static void ReleaseHandles(List<SafeFileHandle> handles)
        {
            if (handles == null) return;
            foreach (var h in handles)
            {
                try { Ioctl(h, NativeMethods.FSCTL_UNLOCK_VOLUME); } catch { }
                h.Dispose();
            }
            handles.Clear();
        }

        public static void UpdateDiskProperties(int n)
        {
            using (var h = Open(PhysicalDrivePath(n), NativeMethods.GENERIC_READ | NativeMethods.GENERIC_WRITE, 0))
            {
                if (!h.IsInvalid) Ioctl(h, NativeMethods.IOCTL_DISK_UPDATE_PROPERTIES);
            }
        }

        /// <summary>
        /// Flush + dismount all volumes and ask the reader to eject the card.
        /// Returns true if the reader confirmed the eject; false means "dismounted, safe to remove".
        /// </summary>
        public static bool SafeEject(int n, Action<string> log)
        {
            var handles = LockAndDismountVolumes(n, log);
            try
            {
                using (var h = Open(PhysicalDrivePath(n), NativeMethods.GENERIC_READ, 0))
                {
                    if (h.IsInvalid) return false;
                    NativeMethods.FlushFileBuffers(h);
                    int r;
                    Ioctl(h, NativeMethods.IOCTL_STORAGE_MEDIA_REMOVAL, new byte[] { 0 }, null, out r);
                    return Ioctl(h, NativeMethods.IOCTL_STORAGE_EJECT_MEDIA);
                }
            }
            finally
            {
                foreach (var h in handles) h.Dispose();
            }
        }

        /// <summary>Wipe the card and create one partition using diskpart. Returns the new drive letter.</summary>
        public static char FormatWithDiskpart(int n, long diskSize, string fileSystem, string label,
            char preferredLetter, Action<string> log, Func<bool> cancelled)
        {
            // Release anything Windows has mounted so diskpart's "clean" doesn't hit "access denied".
            var handles = LockAndDismountVolumes(n, log);
            foreach (var h in handles) h.Dispose();

            string fs = fileSystem.ToLowerInvariant();
            string sizeArg = "";
            if (fs == "fat32" && diskSize > 32000L * 1024 * 1024)
            {
                sizeArg = " size=32000";
                log("  Card is larger than 32 GB: creating a 32 GB FAT32 partition (Windows' FAT32 limit).");
            }
            string id = fs == "fat32" ? "0c" : "07";
            var script = new StringBuilder();
            script.AppendLine("select disk " + n);
            script.AppendLine("clean");
            script.AppendLine("convert mbr");
            script.AppendLine("create partition primary" + sizeArg + " id=" + id);
            script.AppendLine("format fs=" + fs + " quick label=\"" + label + "\"");
            script.AppendLine(preferredLetter != '\0' ? "assign letter=" + preferredLetter : "assign");

            string scriptPath = Path.Combine(Path.GetTempPath(), "sdcardprog_diskpart_" + n + ".txt");
            File.WriteAllText(scriptPath, script.ToString());
            try
            {
                var psi = new ProcessStartInfo("diskpart.exe", "/s \"" + scriptPath + "\"");
                psi.UseShellExecute = false;
                psi.CreateNoWindow = true;
                psi.RedirectStandardOutput = true;
                psi.RedirectStandardError = true;
                using (var p = Process.Start(psi))
                {
                    string output = p.StandardOutput.ReadToEnd();
                    p.WaitForExit(180000);
                    if (!p.HasExited) { p.Kill(); throw new IOException("diskpart timed out."); }
                    if (p.ExitCode != 0)
                    {
                        if (preferredLetter != '\0')
                        {
                            // The old letter may have been grabbed by something else - retry without it.
                            log("  diskpart failed (exit " + p.ExitCode + "); retrying without a fixed drive letter.");
                            return FormatWithDiskpart(n, diskSize, fileSystem, label, '\0', log, cancelled);
                        }
                        throw new IOException("diskpart failed (exit code " + p.ExitCode + "):\r\n" + output.Trim());
                    }
                }
            }
            finally
            {
                try { File.Delete(scriptPath); } catch { }
            }

            // Wait for Windows to mount the new volume.
            for (int i = 0; i < 60; i++)
            {
                if (cancelled()) throw new OperationCanceledException();
                foreach (var kv in LetterMap())
                {
                    if (kv.Value != n) continue;
                    try
                    {
                        var di = new DriveInfo(kv.Key.ToString());
                        if (di.IsReady) return kv.Key;
                    }
                    catch { }
                }
                Thread.Sleep(500);
            }
            throw new IOException("The card was formatted but Windows did not mount it with a drive letter.");
        }
    }
}
