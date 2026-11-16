using System;
using System.Collections.Generic;
using System.IO;
using System.Xml.Serialization;

namespace SdCardProgrammer
{
    /// <summary>A saved job: a friendly name plus everything needed to repeat it.</summary>
    public class Preset
    {
        public string Name = "";
        public JobMode Mode = JobMode.Image;
        public string SourcePath = "";
        public bool Verify = true;
        public bool SkipUnpartitioned = true;
        public bool Format = true;
        public string FileSystem = "FAT32";
        public string Label = "BOOT";
        public bool StripRoot = false;

        public Preset Clone()
        {
            return (Preset)MemberwiseClone();
        }

        public override string ToString()
        {
            return Name + "   (" + (Mode == JobMode.Image ? "image" : "zip") + ": " + Path.GetFileName(SourcePath ?? "") + ")";
        }
    }

    public class AppSettings
    {
        public List<Preset> Presets = new List<Preset>();
        public List<string> RecentFiles = new List<string>();
        public Preset Current = new Preset();
        public string LastPresetName = "";
        /// <summary>Ticked reader ports: a drive letter ("F") or "disk:N" for a reader with no letter.</summary>
        public List<string> SelectedPorts = new List<string>();
        public bool Eject = true;
        public bool AutoStart = false;
        public bool Parallel = true;

        public static string Folder
        {
            get
            {
                return Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "SD Card Programmer");
            }
        }

        public static string FilePath { get { return Path.Combine(Folder, "settings.xml"); } }
        public static string LogPath { get { return Path.Combine(Folder, "log.txt"); } }

        public static AppSettings Load()
        {
            try
            {
                if (File.Exists(FilePath))
                {
                    using (var f = File.OpenRead(FilePath))
                    {
                        var s = (AppSettings)new XmlSerializer(typeof(AppSettings)).Deserialize(f);
                        if (s.Current == null) s.Current = new Preset();
                        return s;
                    }
                }
            }
            catch { }
            return new AppSettings();
        }

        public void Save()
        {
            try
            {
                Directory.CreateDirectory(Folder);
                string tmp = FilePath + ".tmp";
                using (var f = File.Create(tmp))
                    new XmlSerializer(typeof(AppSettings)).Serialize(f, this);
                if (File.Exists(FilePath)) File.Delete(FilePath);
                File.Move(tmp, FilePath);
            }
            catch { }
        }

        public void AddRecent(string path)
        {
            if (string.IsNullOrEmpty(path)) return;
            RecentFiles.RemoveAll(p => string.Equals(p, path, StringComparison.OrdinalIgnoreCase));
            RecentFiles.Insert(0, path);
            while (RecentFiles.Count > 20) RecentFiles.RemoveAt(RecentFiles.Count - 1);
        }

        public Preset FindPreset(string name)
        {
            foreach (var p in Presets)
                if (string.Equals(p.Name, name, StringComparison.OrdinalIgnoreCase)) return p;
            return null;
        }
    }
}
