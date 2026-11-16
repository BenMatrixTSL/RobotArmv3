using System;
using System.Collections.Generic;
using System.IO;
using System.Xml.Serialization;

namespace SdCardProgrammer
{
    [XmlRoot("SdCardJobs")]
    public class JobLibrary
    {
        [XmlElement("Job")]
        public List<Preset> Jobs = new List<Preset>();
    }

    /// <summary>
    /// Where the saved-job list lives. Normally jobs.xml next to the exe, so a folder copied to a
    /// (read-only) network share gives every user the same list. Falls back to the user's AppData
    /// only when the exe's folder can't be written and has no jobs.xml yet.
    /// </summary>
    public class JobStore
    {
        public const string FileName = "jobs.xml";

        public string FilePath;
        public bool BesideExe;
        public bool Writable;

        public static string ExeFolder
        {
            get { return AppDomain.CurrentDomain.BaseDirectory.TrimEnd('\\') + "\\"; }
        }

        public static JobStore Locate()
        {
            string beside = Path.Combine(ExeFolder, FileName);
            if (File.Exists(beside))
                return new JobStore { FilePath = beside, BesideExe = true, Writable = CanWriteFile(beside) };
            if (CanWriteFolder(ExeFolder))
                return new JobStore { FilePath = beside, BesideExe = true, Writable = true };
            return new JobStore { FilePath = Path.Combine(AppSettings.Folder, FileName), BesideExe = false, Writable = true };
        }

        public string Describe()
        {
            return (BesideExe ? "Shared job list: " : "Personal job list: ") + FilePath + (Writable ? "" : "  (read-only)");
        }

        /// <summary>Loads the jobs, with source paths resolved to absolute paths.</summary>
        public List<Preset> Load(out string error)
        {
            error = null;
            var list = new List<Preset>();
            try
            {
                if (!File.Exists(FilePath)) return list;
                using (var f = new FileStream(FilePath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                {
                    var lib = (JobLibrary)new XmlSerializer(typeof(JobLibrary)).Deserialize(f);
                    foreach (var p in lib.Jobs)
                    {
                        if (string.IsNullOrEmpty(p.Name)) continue;
                        p.SourcePath = Resolve(p.SourcePath);
                        list.Add(p);
                    }
                }
            }
            catch (Exception ex)
            {
                error = "Could not read " + FilePath + ": " + (ex.InnerException != null ? ex.InnerException.Message : ex.Message);
            }
            list.Sort((a, b) => string.Compare(a.Name, b.Name, StringComparison.OrdinalIgnoreCase));
            return list;
        }

        /// <summary>Saves the jobs; files inside the exe's folder are stored as relative paths.</summary>
        public void Save(List<Preset> jobs)
        {
            var lib = new JobLibrary();
            foreach (var p in jobs)
            {
                Preset c = p.Clone();
                c.SourcePath = ToStored(c.SourcePath);
                lib.Jobs.Add(c);
            }
            Directory.CreateDirectory(Path.GetDirectoryName(FilePath));
            string tmp = FilePath + ".tmp";
            using (var f = File.Create(tmp))
                new XmlSerializer(typeof(JobLibrary)).Serialize(f, lib);
            if (File.Exists(FilePath)) File.Replace(tmp, FilePath, null);
            else File.Move(tmp, FilePath);
        }

        public static string Resolve(string path)
        {
            if (string.IsNullOrEmpty(path)) return "";
            try
            {
                if (!Path.IsPathRooted(path)) return Path.GetFullPath(Path.Combine(ExeFolder, path));
            }
            catch { }
            return path;
        }

        string ToStored(string path)
        {
            if (!BesideExe || string.IsNullOrEmpty(path)) return path;
            try
            {
                string full = Path.GetFullPath(path);
                if (full.StartsWith(ExeFolder, StringComparison.OrdinalIgnoreCase)) return full.Substring(ExeFolder.Length);
            }
            catch { }
            return path;
        }

        /// <summary>True if a file on this PC's own disks (not the network, not the tool's folder) - other PCs won't see it.</summary>
        public static bool IsLocalOnly(string path)
        {
            try
            {
                string full = Path.GetFullPath(path);
                if (full.StartsWith(@"\\")) return false;
                if (full.StartsWith(ExeFolder, StringComparison.OrdinalIgnoreCase)) return false;
                var drive = new DriveInfo(full.Substring(0, 1));
                return drive.DriveType != DriveType.Network;
            }
            catch { return false; }
        }

        static bool CanWriteFile(string path)
        {
            try
            {
                if ((File.GetAttributes(path) & FileAttributes.ReadOnly) != 0) return false;
                using (new FileStream(path, FileMode.Open, FileAccess.Write, FileShare.ReadWrite)) { }
                return true;
            }
            catch { return false; }
        }

        static bool CanWriteFolder(string folder)
        {
            try
            {
                string probe = Path.Combine(folder, ".write-test-" + Guid.NewGuid().ToString("N"));
                using (new FileStream(probe, FileMode.CreateNew, FileAccess.Write, FileShare.None, 1, FileOptions.DeleteOnClose)) { }
                return true;
            }
            catch { return false; }
        }
    }
}
