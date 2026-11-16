using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Media;
using System.Threading;
using System.Windows.Forms;

namespace SdCardProgrammer
{
    public class MainForm : Form
    {
        enum UiState { Idle, Running, Done, Failed, Waiting }

        /// <summary>One card-reader port in the list.</summary>
        class PortItem
        {
            public DiskInfo Disk;
            public string Status = "";
            public string Key { get { return PortKey(Disk); } }
            public override string ToString()
            {
                return Disk.Describe() + (Status.Length > 0 ? "      " + Status : "");
            }
        }

        static string PortKey(DiskInfo d)
        {
            return d.Letters.Count > 0 ? d.Letters[0].ToString() : "disk:" + d.Number;
        }

        static string PortName(DiskInfo d)
        {
            return d.Letters.Count > 0 ? d.Letters[0] + ":" : "Disk " + d.Number;
        }

        readonly AppSettings settings;
        float scale = 1f;

        ComboBox cboPreset, cboSource, cboFs;
        CheckedListBox lstPorts;
        Button btnSavePreset, btnDeletePreset, btnRefresh, btnBrowse, btnStart, btnNext, btnCancel;
        RadioButton rbImage, rbZip;
        CheckBox chkVerify, chkSkip, chkFormat, chkStrip, chkEject, chkAuto;
        TextBox txtLabel, txtLog;
        Label lblBig, lblSmall, lblDetail, lblCounter;
        TableLayoutPanel pnlStatus;
        ProgressBar progress;
        readonly List<Control> lockWhileRunning = new List<Control>();
        System.Windows.Forms.Timer pollTimer, refreshTimer;

        UiState state = UiState.Idle;
        JobContext currentJob;
        Thread worker;
        bool loadingUi;
        int okCount, failCount;

        // Batch bookkeeping (all keyed by physical disk number)
        readonly HashSet<int> confirmedDisks = new HashSet<int>();   // user agreed these may be erased
        readonly HashSet<int> needsSwap = new HashSet<int>();        // finished card still in the port
        readonly Dictionary<int, string> portStatus = new Dictionary<int, string>();
        readonly Dictionary<int, int> presentStreak = new Dictionary<int, int>();
        bool hadJob;
        string lastMediaSignature = "";

        // progress / speed tracking
        string curPhase = "", curPhaseWorker = "";
        DateTime phaseStart, lastUiProgress = DateTime.MinValue;
        int batchIndex, batchCount, batchDisk = -1;
        string batchPort = "";

        public MainForm()
        {
            settings = AppSettings.Load();
            Text = "SD Card Programmer";
            Font = new Font("Segoe UI", 10f);
            using (Graphics g = CreateGraphics()) scale = g.DpiX / 96f;
            ClientSize = new Size(S(1000), S(940));
            MinimumSize = new Size(S(840), S(780));
            StartPosition = FormStartPosition.CenterScreen;
            try { Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath); } catch { }

            BuildUi();
            LoadSettingsIntoUi();

            pollTimer = new System.Windows.Forms.Timer { Interval = 1000 };
            pollTimer.Tick += delegate { Poll(); };
            pollTimer.Start();
            refreshTimer = new System.Windows.Forms.Timer { Interval = 800 };
            refreshTimer.Tick += delegate { refreshTimer.Stop(); RefreshPorts(); };

            SetStatus(UiState.Idle, "READY", "Tick the card ports to program, choose a job, then press START.");
            Log("SD Card Programmer started. Settings: " + AppSettings.FilePath);
            if (ImageSource.FindSevenZip() == null) Log("7-Zip not found: .xz/.7z images are unavailable (plain .img, .gz and .zip are fine).");
        }

        int S(int px) { return (int)Math.Round(px * scale); }

        // ================================================================== UI construction

        void BuildUi()
        {
            var grid = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, Padding = new Padding(S(12)) };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            Controls.Add(grid);

            // Saved jobs
            cboPreset = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Dock = DockStyle.Fill };
            cboPreset.SelectedIndexChanged += delegate { if (!loadingUi && cboPreset.SelectedItem is Preset) ApplyPreset((Preset)cboPreset.SelectedItem); };
            btnSavePreset = new Button { Text = "Save as...", AutoSize = true };
            btnSavePreset.Click += delegate { SavePreset(); };
            btnDeletePreset = new Button { Text = "Delete", AutoSize = true };
            btnDeletePreset.Click += delegate { DeletePreset(); };
            AddRow(grid, "Saved job:", cboPreset, Flow(btnSavePreset, btnDeletePreset));

            // Ports
            lstPorts = new CheckedListBox { Dock = DockStyle.Fill, CheckOnClick = true, IntegralHeight = false, Height = S(118) };
            lstPorts.ItemCheck += delegate { if (!loadingUi) BeginInvoke((Action)SaveSelectedPorts); };
            btnRefresh = new Button { Text = "Refresh", AutoSize = true, Anchor = AnchorStyles.Top };
            btnRefresh.Click += delegate { RefreshPorts(); };
            var portLabel = AddRow(grid, "Card ports:", lstPorts, btnRefresh);
            portLabel.Anchor = AnchorStyles.Left | AnchorStyles.Top;
            portLabel.Text = "Card ports:\n(tick each\nport to\nprogram)";

            // Mode
            rbImage = new RadioButton { Text = "Clone disk image (.img) onto the whole card", AutoSize = true, Checked = true };
            rbZip = new RadioButton { Text = "Copy ZIP contents onto the card", AutoSize = true };
            rbImage.CheckedChanged += delegate { UpdateModeEnables(); };
            AddRow(grid, "Job type:", Flow(rbImage, rbZip), null);

            // Source
            cboSource = new ComboBox { DropDownStyle = ComboBoxStyle.DropDown, Dock = DockStyle.Fill };
            btnBrowse = new Button { Text = "Browse...", AutoSize = true };
            btnBrowse.Click += delegate { Browse(); };
            AddRow(grid, "Source file:", cboSource, btnBrowse);

            // Image options
            chkVerify = new CheckBox { Text = "Verify after writing", AutoSize = true, Checked = true };
            chkSkip = new CheckBox { Text = "Skip unused space after the last partition (faster)", AutoSize = true, Checked = true };
            AddRow(grid, "Image options:", Flow(chkVerify, chkSkip), null);

            // Zip options
            chkFormat = new CheckBox { Text = "Format card first", AutoSize = true, Checked = true };
            chkFormat.CheckedChanged += delegate { UpdateModeEnables(); };
            cboFs = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = S(90) };
            cboFs.Items.AddRange(new object[] { "FAT32", "exFAT", "NTFS" });
            cboFs.SelectedIndex = 0;
            txtLabel = new TextBox { Width = S(110), MaxLength = 11, CharacterCasing = CharacterCasing.Upper };
            chkStrip = new CheckBox { Text = "Remove single top folder", AutoSize = true };
            AddRow(grid, "ZIP options:", Flow(chkFormat, SmallLabel("as"), cboFs, SmallLabel("label"), txtLabel, chkStrip), null);

            // After
            chkEject = new CheckBox { Text = "Safely eject each card when finished", AutoSize = true, Checked = true };
            chkAuto = new CheckBox { Text = "Auto-start when every ticked port has a new card", AutoSize = true };
            AddRow(grid, "When done:", Flow(chkEject, chkAuto), null);

            // Action buttons
            var big = new Font("Segoe UI", 14f, FontStyle.Bold);
            btnStart = new Button { Text = "START", Font = big, Size = new Size(S(160), S(58)), BackColor = Color.FromArgb(0x15, 0x65, 0xC0), ForeColor = Color.White, FlatStyle = FlatStyle.Flat };
            btnStart.Click += delegate { StartBatch(true); };
            btnNext = new Button { Text = "PROGRAM NEXT BATCH", Font = big, Size = new Size(S(300), S(58)), BackColor = Color.FromArgb(0x1E, 0x9E, 0x3A), ForeColor = Color.White, FlatStyle = FlatStyle.Flat, Enabled = false };
            btnNext.Click += delegate { NextBatch(); };
            btnCancel = new Button { Text = "Cancel", Font = new Font("Segoe UI", 12f), Size = new Size(S(120), S(58)), Enabled = false };
            btnCancel.Click += delegate { CancelClicked(); };
            lblCounter = new Label { AutoSize = true, Font = new Font("Segoe UI", 11f), Margin = new Padding(S(16), S(18), 0, 0) };
            var buttons = Flow(btnStart, btnNext, btnCancel, lblCounter);
            buttons.Margin = new Padding(0, S(10), 0, S(6));
            AddSpan(grid, buttons, SizeType.AutoSize, 0);

            // Big status indicator
            pnlStatus = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2, Margin = new Padding(0, S(4), 0, S(4)) };
            pnlStatus.RowStyles.Add(new RowStyle(SizeType.Percent, 60));
            pnlStatus.RowStyles.Add(new RowStyle(SizeType.Percent, 40));
            lblBig = new Label { Dock = DockStyle.Fill, TextAlign = ContentAlignment.BottomCenter, Font = new Font("Segoe UI", 32f, FontStyle.Bold) };
            lblSmall = new Label { Dock = DockStyle.Fill, TextAlign = ContentAlignment.TopCenter, Font = new Font("Segoe UI", 13f) };
            pnlStatus.Controls.Add(lblBig, 0, 0);
            pnlStatus.Controls.Add(lblSmall, 0, 1);
            AddSpan(grid, pnlStatus, SizeType.Absolute, S(170));

            progress = new ProgressBar { Dock = DockStyle.Fill, Height = S(26), Maximum = 1000 };
            AddSpan(grid, progress, SizeType.AutoSize, 0);
            lblDetail = new Label { Dock = DockStyle.Fill, AutoSize = true, Margin = new Padding(0, S(2), 0, S(6)) };
            AddSpan(grid, lblDetail, SizeType.AutoSize, 0);

            txtLog = new TextBox { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Vertical, Dock = DockStyle.Fill, Font = new Font("Consolas", 9f), BackColor = SystemColors.Window };
            AddSpan(grid, txtLog, SizeType.Percent, 100);

            lockWhileRunning.AddRange(new Control[] { cboPreset, btnSavePreset, btnDeletePreset, lstPorts, btnRefresh, rbImage, rbZip,
                cboSource, btnBrowse, chkVerify, chkSkip, chkFormat, cboFs, txtLabel, chkStrip, chkEject, btnStart });
            UpdateCounter();
        }

        Label SmallLabel(string text)
        {
            return new Label { Text = text, AutoSize = true, Margin = new Padding(S(6), S(6), 0, 0) };
        }

        FlowLayoutPanel Flow(params Control[] controls)
        {
            var f = new FlowLayoutPanel { AutoSize = true, AutoSizeMode = AutoSizeMode.GrowAndShrink, WrapContents = false, Anchor = AnchorStyles.Left, Margin = new Padding(0) };
            foreach (var c in controls) f.Controls.Add(c);
            return f;
        }

        Label AddRow(TableLayoutPanel grid, string label, Control main, Control right)
        {
            int row = grid.RowCount++;
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            var l = new Label { Text = label, AutoSize = true, Anchor = AnchorStyles.Left, Font = new Font(Font, FontStyle.Bold), Margin = new Padding(0, S(4), S(10), S(4)) };
            grid.Controls.Add(l, 0, row);
            main.Margin = new Padding(0, S(4), S(6), S(4));
            grid.Controls.Add(main, 1, row);
            if (right != null) grid.Controls.Add(right, 2, row);
            return l;
        }

        void AddSpan(TableLayoutPanel grid, Control c, SizeType type, float size)
        {
            int row = grid.RowCount++;
            grid.RowStyles.Add(type == SizeType.AutoSize ? new RowStyle(SizeType.AutoSize) : new RowStyle(type, size));
            grid.Controls.Add(c, 0, row);
            grid.SetColumnSpan(c, 3);
        }

        void UpdateModeEnables()
        {
            bool running = state == UiState.Running;
            bool image = rbImage.Checked;
            chkVerify.Enabled = image && !running;
            chkSkip.Enabled = image && !running;
            chkFormat.Enabled = !image && !running;
            cboFs.Enabled = !image && chkFormat.Checked && !running;
            txtLabel.Enabled = !image && chkFormat.Checked && !running;
            chkStrip.Enabled = !image && !running;
        }

        // ================================================================== settings & presets

        void LoadSettingsIntoUi()
        {
            loadingUi = true;
            RefreshPresetList(settings.LastPresetName);
            loadingUi = false;
            ApplyPreset(settings.Current);
            chkEject.Checked = settings.Eject;
            chkAuto.Checked = settings.AutoStart;
            RefreshPorts();
        }

        void RefreshPresetList(string selectName)
        {
            bool was = loadingUi;
            loadingUi = true;
            cboPreset.Items.Clear();
            settings.Presets.Sort((a, b) => string.Compare(a.Name, b.Name, StringComparison.OrdinalIgnoreCase));
            foreach (var p in settings.Presets) cboPreset.Items.Add(p);
            cboPreset.SelectedItem = settings.FindPreset(selectName);
            loadingUi = was;
        }

        void RefreshRecentList()
        {
            string text = cboSource.Text;
            cboSource.Items.Clear();
            foreach (string f in settings.RecentFiles) cboSource.Items.Add(f);
            cboSource.Text = text;
        }

        void ApplyPreset(Preset p)
        {
            rbImage.Checked = p.Mode == JobMode.Image;
            rbZip.Checked = p.Mode == JobMode.Zip;
            RefreshRecentList();
            cboSource.Text = p.SourcePath ?? "";
            chkVerify.Checked = p.Verify;
            chkSkip.Checked = p.SkipUnpartitioned;
            chkFormat.Checked = p.Format;
            int fs = cboFs.Items.IndexOf(p.FileSystem ?? "FAT32");
            cboFs.SelectedIndex = fs < 0 ? 0 : fs;
            txtLabel.Text = p.Label ?? "";
            chkStrip.Checked = p.StripRoot;
            UpdateModeEnables();
            if (!string.IsNullOrEmpty(p.Name))
            {
                settings.LastPresetName = p.Name;
                Log("Loaded saved job '" + p.Name + "'.");
                if (!string.IsNullOrEmpty(p.SourcePath) && !File.Exists(p.SourcePath))
                    Log("WARNING: the file for this job is missing: " + p.SourcePath);
            }
        }

        Preset ReadUiPreset()
        {
            return new Preset
            {
                Mode = rbImage.Checked ? JobMode.Image : JobMode.Zip,
                SourcePath = cboSource.Text.Trim().Trim('"'),
                Verify = chkVerify.Checked,
                SkipUnpartitioned = chkSkip.Checked,
                Format = chkFormat.Checked,
                FileSystem = (string)cboFs.SelectedItem ?? "FAT32",
                Label = txtLabel.Text.Trim(),
                StripRoot = chkStrip.Checked,
            };
        }

        void SaveUiToSettings()
        {
            settings.Current = ReadUiPreset();
            settings.Eject = chkEject.Checked;
            settings.AutoStart = chkAuto.Checked;
            settings.Save();
        }

        void SavePreset()
        {
            Preset p = ReadUiPreset();
            if (string.IsNullOrEmpty(p.SourcePath)) { Warn("Choose a source file first."); return; }
            string suggested = cboPreset.SelectedItem is Preset ? ((Preset)cboPreset.SelectedItem).Name : Path.GetFileNameWithoutExtension(p.SourcePath);
            string name = InputBox.Show(this, "Save job", "Friendly name for this job (e.g. \"Robot Arm Pi image v3\"):", suggested);
            if (string.IsNullOrEmpty(name)) return;
            Preset existing = settings.FindPreset(name);
            if (existing != null)
            {
                if (MessageBox.Show(this, "Replace the saved job '" + existing.Name + "'?", "Save job", MessageBoxButtons.YesNo, MessageBoxIcon.Question) != DialogResult.Yes) return;
                settings.Presets.Remove(existing);
            }
            p.Name = name;
            settings.Presets.Add(p);
            settings.LastPresetName = name;
            settings.AddRecent(p.SourcePath);
            SaveUiToSettings();
            RefreshPresetList(name);
            RefreshRecentList();
            Log("Saved job '" + name + "'.");
        }

        void DeletePreset()
        {
            var p = cboPreset.SelectedItem as Preset;
            if (p == null) return;
            if (MessageBox.Show(this, "Delete the saved job '" + p.Name + "'?\n(The file itself is not deleted.)", "Delete job", MessageBoxButtons.YesNo, MessageBoxIcon.Question) != DialogResult.Yes) return;
            settings.Presets.Remove(p);
            settings.LastPresetName = "";
            settings.Save();
            RefreshPresetList("");
        }

        void Browse()
        {
            using (var dlg = new OpenFileDialog())
            {
                if (rbImage.Checked)
                    dlg.Filter = "Disk images (*.img, *.gz, *.xz, *.zip, *.iso, *.bin, *.7z)|*.img;*.gz;*.xz;*.zip;*.iso;*.bin;*.7z|All files (*.*)|*.*";
                else
                    dlg.Filter = "ZIP files (*.zip)|*.zip|All files (*.*)|*.*";
                string cur = cboSource.Text.Trim('"');
                try { if (File.Exists(cur)) dlg.InitialDirectory = Path.GetDirectoryName(cur); } catch { }
                if (dlg.ShowDialog(this) == DialogResult.OK) cboSource.Text = dlg.FileName;
            }
        }

        // ================================================================== ports

        List<DiskInfo> CheckedDisks()
        {
            var list = new List<DiskInfo>();
            foreach (object o in lstPorts.CheckedItems) list.Add(((PortItem)o).Disk);
            return list;
        }

        void SaveSelectedPorts()
        {
            settings.SelectedPorts.Clear();
            foreach (var d in CheckedDisks()) settings.SelectedPorts.Add(PortKey(d));
            settings.Save();
        }

        void RefreshPorts()
        {
            List<DiskInfo> disks;
            try { disks = DiskTools.ListCandidateDisks(); }
            catch (Exception ex) { Log("Drive scan failed: " + ex.Message); return; }

            // During a batch the disk list is frozen (the job holds the disk numbers); just refresh text.
            if (state == UiState.Running) { RedrawPortStatus(); return; }

            loadingUi = true;
            int top = lstPorts.TopIndex;
            lstPorts.BeginUpdate();
            lstPorts.Items.Clear();
            foreach (var d in disks)
            {
                string status;
                portStatus.TryGetValue(d.Number, out status);
                int i = lstPorts.Items.Add(new PortItem { Disk = d, Status = status ?? "" });
                lstPorts.SetItemChecked(i, settings.SelectedPorts.Contains(PortKey(d)));
            }
            if (top < lstPorts.Items.Count) lstPorts.TopIndex = top;
            lstPorts.EndUpdate();
            loadingUi = false;
            lastMediaSignature = MediaSignature(disks);
        }

        void RedrawPortStatus()
        {
            for (int i = 0; i < lstPorts.Items.Count; i++)
            {
                var item = (PortItem)lstPorts.Items[i];
                string s;
                portStatus.TryGetValue(item.Disk.Number, out s);
                s = s ?? "";
                if (s != item.Status)
                {
                    item.Status = s;
                    bool chk = lstPorts.GetItemChecked(i);
                    loadingUi = true;
                    lstPorts.Items[i] = item;           // forces the row to repaint
                    lstPorts.SetItemChecked(i, chk);
                    loadingUi = false;
                }
            }
        }

        static string MediaSignature(List<DiskInfo> disks)
        {
            var sb = new System.Text.StringBuilder();
            foreach (var d in disks) sb.Append(d.Number).Append(d.HasMedia ? '+' : '-');
            return sb.ToString();
        }

        protected override void WndProc(ref Message m)
        {
            base.WndProc(ref m);
            const int WM_DEVICECHANGE = 0x219;
            if (m.Msg == WM_DEVICECHANGE && refreshTimer != null)
            {
                refreshTimer.Stop();
                refreshTimer.Start();
            }
        }

        void Poll()
        {
            // Track card removal/insertion on every listed port.
            var sig = new System.Text.StringBuilder();
            var present = new Dictionary<int, bool>();
            foreach (object o in lstPorts.Items)
            {
                var d = ((PortItem)o).Disk;
                bool p = DiskTools.MediaPresent(d.Number);
                present[d.Number] = p;
                sig.Append(d.Number).Append(p ? '+' : '-');
                int streak;
                presentStreak.TryGetValue(d.Number, out streak);
                presentStreak[d.Number] = p ? streak + 1 : 0;
                if (!p && needsSwap.Remove(d.Number))
                {
                    portStatus[d.Number] = "removed - insert next card";
                }
                else if (!p && portStatus.ContainsKey(d.Number) && portStatus[d.Number].StartsWith("✔") && state != UiState.Running)
                {
                    portStatus[d.Number] = "removed - insert next card";
                }
                if (p && !needsSwap.Contains(d.Number) && portStatus.ContainsKey(d.Number) && portStatus[d.Number].StartsWith("removed"))
                    portStatus[d.Number] = "new card ready";
            }
            if (state != UiState.Running && sig.ToString() != lastMediaSignature) RefreshPorts();
            else RedrawPortStatus();

            if (state == UiState.Running || !hadJob) return;
            if (state != UiState.Waiting && !(state == UiState.Done && chkAuto.Checked)) return;

            // Ready when every ticked port holds a card that isn't one we just finished.
            List<DiskInfo> ticked = CheckedDisks();
            if (ticked.Count == 0) return;
            int ready = 0;
            var waitingFor = new List<string>();
            foreach (var d in ticked)
            {
                bool p;
                present.TryGetValue(d.Number, out p);
                int streak;
                presentStreak.TryGetValue(d.Number, out streak);
                if (needsSwap.Contains(d.Number)) waitingFor.Add(PortName(d) + " swap card");
                else if (!p) waitingFor.Add(PortName(d) + " insert card");
                else if (streak >= 2) ready++;
                else waitingFor.Add(PortName(d) + " settling");
            }
            if (ready == ticked.Count)
            {
                StartBatch(false);
                return;
            }
            if (state == UiState.Waiting)
                SetStatus(UiState.Waiting, "INSERT NEXT CARDS  (" + ready + " of " + ticked.Count + " ready)",
                    "Waiting for: " + string.Join(",  ", waitingFor.ToArray()) + "      (Cancel to stop waiting)");
        }

        // ================================================================== running jobs

        void NextBatch()
        {
            SetStatus(UiState.Waiting, "INSERT NEXT CARDS", "Swap the cards in the ticked ports; programming starts when they are all in.");
            Poll();
        }

        void CancelClicked()
        {
            if (state == UiState.Running && currentJob != null)
            {
                currentJob.CancelRequested = true;
                btnCancel.Enabled = false;
                Log("Cancelling...");
            }
            else if (state == UiState.Waiting)
            {
                SetStatus(UiState.Idle, "READY", "Stopped waiting. Press START or PROGRAM NEXT BATCH.");
            }
        }

        void StartBatch(bool confirm)
        {
            if (state == UiState.Running) return;
            Preset p = ReadUiPreset();
            if (string.IsNullOrEmpty(p.SourcePath) || !File.Exists(p.SourcePath)) { Fail("Source file not found:\n" + p.SourcePath); return; }
            if (p.Mode == JobMode.Zip && !p.SourcePath.EndsWith(".zip", StringComparison.OrdinalIgnoreCase)) { Fail("ZIP mode needs a .zip file."); return; }
            if (p.Mode == JobMode.Zip && p.Format)
            {
                p.Label = SanitizeLabel(p.Label, p.FileSystem);
                txtLabel.Text = p.Label;
            }

            List<DiskInfo> ticked = CheckedDisks();
            if (ticked.Count == 0) { Fail("Tick at least one card port to program."); return; }

            // Re-check every port right now; skip empty ones.
            int systemDisk = DiskTools.SystemDiskNumber();
            var targets = new List<DiskInfo>();
            var empty = new List<string>();
            foreach (var t in ticked)
            {
                DiskInfo d = DiskTools.QueryDisk(t.Number);
                string reason;
                if (!DiskTools.IsSafeTarget(d, systemDisk, out reason)) { Fail(PortName(t) + ": " + reason); RefreshPorts(); return; }
                d.Letters = t.Letters;
                if (d.HasMedia) targets.Add(d); else empty.Add(PortName(t));
            }
            if (targets.Count == 0) { Fail("None of the ticked ports has a card in it."); return; }

            bool allConfirmed = true;
            foreach (var d in targets) if (!confirmedDisks.Contains(d.Number)) allConfirmed = false;
            if (confirm || !allConfirmed)
            {
                var sb = new System.Text.StringBuilder();
                sb.AppendLine(p.Mode == JobMode.Zip && !p.Format
                    ? "Files will be copied onto these cards (same-named files are overwritten):"
                    : "ALL DATA on these cards will be erased:");
                sb.AppendLine();
                long largest = 0;
                foreach (var d in targets) { sb.AppendLine("    " + d.Describe()); largest = Math.Max(largest, d.Size); }
                if (empty.Count > 0) { sb.AppendLine(); sb.AppendLine("No card (skipped): " + string.Join(", ", empty.ToArray())); }
                sb.AppendLine();
                sb.AppendLine("Job: " + (p.Mode == JobMode.Image ? "clone image " : "copy ZIP ") + Path.GetFileName(p.SourcePath));
                sb.AppendLine();
                sb.Append("Continue?");
                if (MessageBox.Show(this, sb.ToString(), "Confirm SD cards", MessageBoxButtons.YesNo, MessageBoxIcon.Warning) != DialogResult.Yes) return;
                if (largest > DiskTools.LargeDiskWarningBytes &&
                    MessageBox.Show(this, "One of these disks is " + DiskTools.FormatSize(largest) + " - unusually large for an SD card.\nAre you SURE these are the right devices?",
                        "Large disk", MessageBoxButtons.YesNo, MessageBoxIcon.Exclamation, MessageBoxDefaultButton.Button2) != DialogResult.Yes) return;
                foreach (var d in targets) confirmedDisks.Add(d.Number);
            }
            else if (empty.Count > 0)
            {
                Log("No card in " + string.Join(", ", empty.ToArray()) + " - skipped.");
            }

            settings.AddRecent(p.SourcePath);
            SaveUiToSettings();
            RefreshRecentList();

            foreach (var d in ticked) portStatus.Remove(d.Number);
            foreach (var d in targets) { portStatus[d.Number] = "queued"; needsSwap.Remove(d.Number); }
            foreach (var e in ticked) if (!targets.Exists(t => t.Number == e.Number)) portStatus[e.Number] = "no card - skipped";
            RedrawPortStatus();

            bool eject = chkEject.Checked;
            var ctx = new JobContext();
            ctx.Log = msg => BeginInvoke((Action)(() => Log(msg)));
            ctx.Progress = (phase, done, total, frac) =>
            {
                DateTime now = DateTime.Now;
                if (phase == curPhaseWorker && (now - lastUiProgress).TotalMilliseconds < 150 && done != total) return;
                lastUiProgress = now;
                curPhaseWorker = phase;
                BeginInvoke((Action)(() => ShowProgress(phase, done, total, frac)));
            };
            currentJob = ctx;
            hadJob = true;

            Log("========== Batch started: " + Path.GetFileName(p.SourcePath) + " -> " + targets.Count + " card(s)");
            SetStatus(UiState.Running, "STARTING...", "");
            progress.Value = 0;
            lblDetail.Text = "";
            DateTime started = DateTime.Now;

            worker = new Thread(() =>
            {
                var failures = new List<string>();
                int ok = 0;
                bool cancelled = false;
                for (int i = 0; i < targets.Count && !cancelled; i++)
                {
                    DiskInfo d = targets[i];
                    int idx = i;
                    BeginInvoke((Action)(() => CardStarting(idx, targets.Count, d)));
                    var opts = new JobOptions
                    {
                        Mode = p.Mode, SourcePath = p.SourcePath, DiskNumber = d.Number,
                        Verify = p.Verify, SkipUnpartitioned = p.SkipUnpartitioned,
                        Format = p.Format, FileSystem = p.FileSystem, Label = p.Label, StripRoot = p.StripRoot,
                        Eject = eject,
                    };
                    DateTime t0 = DateTime.Now;
                    try
                    {
                        JobResult r = Jobs.Run(opts, ctx);
                        ok++;
                        string took = FormatDuration(DateTime.Now - t0);
                        BeginInvoke((Action)(() => CardFinished(d, true, r.Summary + " in " + took, r.Ejected, eject)));
                    }
                    catch (OperationCanceledException)
                    {
                        cancelled = true;
                        BeginInvoke((Action)(() => CardFinished(d, false, "cancelled", false, eject)));
                    }
                    catch (Exception ex)
                    {
                        failures.Add(PortName(d) + ": " + ex.Message);
                        string msg = ex.Message;
                        BeginInvoke((Action)(() => CardFinished(d, false, msg, false, eject)));
                    }
                }
                TimeSpan total = DateTime.Now - started;
                try { BeginInvoke((Action)(() => BatchFinished(ok, targets.Count, failures, cancelled, eject, total))); } catch { }
            });
            worker.IsBackground = true;
            worker.Start();
        }

        void CardStarting(int index, int count, DiskInfo d)
        {
            batchIndex = index;
            batchCount = count;
            batchPort = PortName(d);
            batchDisk = d.Number;
            curPhase = "";
            curPhaseWorker = "";
            portStatus[d.Number] = "▶ programming...";
            RedrawPortStatus();
            lblSmall.Text = CardPrefix() + d.Model + ", " + DiskTools.FormatSize(d.Size);
            progress.Value = 0;
        }

        string CardPrefix()
        {
            return batchCount > 1 ? "Card " + (batchIndex + 1) + " of " + batchCount + "  (" + batchPort + ")   -   " : batchPort + "   -   ";
        }

        void CardFinished(DiskInfo d, bool ok, string message, bool ejected, bool ejectRequested)
        {
            if (ok)
            {
                okCount++;
                needsSwap.Add(d.Number);
                portStatus[d.Number] = "✔ DONE" + (ejectRequested ? " - remove card" : "");
                Log(PortName(d) + " SUCCESS: " + message + (ejectRequested ? (ejected ? " - ejected." : " - unmounted, safe to remove.") : ""));
            }
            else if (message == "cancelled")
            {
                portStatus[d.Number] = "✖ cancelled - card incomplete";
                Log(PortName(d) + " CANCELLED - card incomplete.");
            }
            else
            {
                failCount++;
                portStatus[d.Number] = "✖ FAILED";
                Log(PortName(d) + " FAILED: " + message);
            }
            RedrawPortStatus();
            UpdateCounter();
        }

        void ShowProgress(string phase, long done, long total, double frac)
        {
            if (state != UiState.Running) return;
            if (phase != curPhase) { curPhase = phase; phaseStart = DateTime.Now; }
            double f = total > 0 ? (double)done / total : frac;
            bool measured = phase == "Writing" || phase == "Verifying" || phase == "Copying";
            if (!measured)
            {
                lblBig.Text = phase.TrimEnd('.').ToUpperInvariant() + "...";
                progress.Style = ProgressBarStyle.Marquee;
                lblDetail.Text = "";
                return;
            }
            progress.Style = ProgressBarStyle.Continuous;
            string big = phase.ToUpperInvariant();
            if (f >= 0) { progress.Value = Math.Max(0, Math.Min(1000, (int)(f * 1000))); big += "  " + (int)(f * 100) + "%"; }
            lblBig.Text = big;

            double secs = (DateTime.Now - phaseStart).TotalSeconds;
            string detail = DiskTools.FormatSize(done) + (total > 0 ? " of " + DiskTools.FormatSize(total) : "");
            if (secs > 2)
            {
                double rate = done / secs;
                detail += "   -   " + (rate / 1e6).ToString("0.0") + " MB/s";
                if (f > 0.01 && f < 1) detail += "   -   about " + FormatDuration(TimeSpan.FromSeconds(secs / f - secs)) + " left on this card";
            }
            lblDetail.Text = detail;
            string pct = f >= 0 ? (int)(f * 100) + "% " : "";
            if (batchDisk >= 0) portStatus[batchDisk] = "▶ " + phase.ToLowerInvariant() + " " + pct;
        }

        void BatchFinished(int ok, int count, List<string> failures, bool cancelled, bool ejectRequested, TimeSpan took)
        {
            currentJob = null;
            progress.Style = ProgressBarStyle.Continuous;
            foreach (var k in new List<int>(portStatus.Keys))
                if (portStatus[k] == "queued") portStatus[k] = "not started";
            string next = chkAuto.Checked
                ? "Swap the cards - the next batch starts automatically."
                : "Swap the cards and press PROGRAM NEXT BATCH.";
            string removal = ejectRequested ? "Safe to remove. " : "Eject the cards in Windows before removing them. ";

            if (cancelled)
            {
                SetStatus(UiState.Failed, "CANCELLED", ok + " of " + count + " cards finished. Any card not marked DONE must be programmed again.");
                Log("Batch cancelled after " + ok + " of " + count + " card(s).");
            }
            else if (failures.Count == 0)
            {
                progress.Value = 1000;
                SetStatus(UiState.Done, "✔  COMPLETE" + (count > 1 ? "  -  " + count + " CARDS" : ""), removal + next);
                lblDetail.Text = "Batch finished in " + FormatDuration(took) + ".";
                Log("BATCH COMPLETE: " + count + " card(s) in " + FormatDuration(took) + ".");
                SystemSounds.Asterisk.Play();
            }
            else
            {
                SetStatus(UiState.Failed, "✖  " + failures.Count + " OF " + count + " FAILED",
                    Shorten(string.Join("   ", failures.ToArray())) + (ok > 0 ? "   (" + ok + " OK - safe to remove)" : ""));
                Log("BATCH FINISHED WITH ERRORS: " + ok + " ok, " + failures.Count + " failed.");
                SystemSounds.Hand.Play();
            }
            UpdateCounter();
            RefreshPorts();
        }

        void SetStatus(UiState s, string big, string small)
        {
            state = s;
            Color back, fore = Color.White;
            switch (s)
            {
                case UiState.Running: back = Color.FromArgb(0xF2, 0xA9, 0x00); fore = Color.Black; break;
                case UiState.Done: back = Color.FromArgb(0x1E, 0x9E, 0x3A); break;
                case UiState.Failed: back = Color.FromArgb(0xC6, 0x28, 0x28); break;
                case UiState.Waiting: back = Color.FromArgb(0x15, 0x65, 0xC0); break;
                default: back = Color.FromArgb(0x5F, 0x63, 0x68); break;
            }
            pnlStatus.BackColor = back;
            lblBig.ForeColor = lblSmall.ForeColor = fore;
            lblBig.Text = big;
            lblSmall.Text = small;

            bool running = s == UiState.Running;
            foreach (var c in lockWhileRunning) c.Enabled = !running;
            UpdateModeEnables();
            btnNext.Enabled = !running && hadJob && s != UiState.Waiting;
            btnNext.BackColor = btnNext.Enabled ? Color.FromArgb(0x1E, 0x9E, 0x3A) : Color.FromArgb(0xB8, 0xC4, 0xBB);
            btnStart.BackColor = btnStart.Enabled ? Color.FromArgb(0x15, 0x65, 0xC0) : Color.FromArgb(0xB0, 0xBE, 0xD0);
            btnCancel.Enabled = running || s == UiState.Waiting;
        }

        void UpdateCounter()
        {
            lblCounter.Text = "This session:  " + okCount + " cards done" + (failCount > 0 ? ",  " + failCount + " failed" : "");
        }

        void Fail(string message)
        {
            Log("Cannot start: " + message.Replace("\n", " "));
            SetStatus(UiState.Idle, "READY", "Fix the problem above, then press START.");
            MessageBox.Show(this, message, "SD Card Programmer", MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }

        void Warn(string message)
        {
            MessageBox.Show(this, message, "SD Card Programmer", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }

        static string SanitizeLabel(string label, string fs)
        {
            var sb = new System.Text.StringBuilder();
            foreach (char c in (label ?? "").ToUpperInvariant())
                if (char.IsLetterOrDigit(c) || c == '_' || c == '-' || c == ' ') sb.Append(c);
            string s = sb.ToString().Trim();
            int max = fs == "NTFS" ? 32 : 11;
            if (s.Length > max) s = s.Substring(0, max);
            return s.Length == 0 ? "SDCARD" : s;
        }

        static string Shorten(string s)
        {
            s = (s ?? "").Replace("\r", " ").Replace("\n", " ");
            return s.Length > 200 ? s.Substring(0, 197) + "..." : s;
        }

        static string FormatDuration(TimeSpan t)
        {
            if (t.TotalSeconds < 60) return (int)t.TotalSeconds + " s";
            if (t.TotalMinutes < 60) return (int)t.TotalMinutes + " min " + t.Seconds + " s";
            return (int)t.TotalHours + " h " + t.Minutes + " min";
        }

        void Log(string msg)
        {
            string line = DateTime.Now.ToString("HH:mm:ss") + "  " + msg;
            txtLog.AppendText(line + "\r\n");
            try
            {
                Directory.CreateDirectory(AppSettings.Folder);
                File.AppendAllText(AppSettings.LogPath, DateTime.Now.ToString("yyyy-MM-dd ") + line + "\r\n");
            }
            catch { }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            if (state == UiState.Running)
            {
                if (MessageBox.Show(this, "Cards are being programmed. Stop and exit? The current card will be left incomplete.", "Exit",
                        MessageBoxButtons.YesNo, MessageBoxIcon.Warning) != DialogResult.Yes)
                {
                    e.Cancel = true;
                    return;
                }
                if (currentJob != null) currentJob.CancelRequested = true;
                if (worker != null) worker.Join(10000);
            }
            SaveUiToSettings();
            base.OnFormClosing(e);
        }
    }

    /// <summary>Minimal text-prompt dialog.</summary>
    static class InputBox
    {
        public static string Show(IWin32Window owner, string title, string prompt, string value)
        {
            using (var f = new Form())
            {
                f.Text = title;
                f.Font = new Font("Segoe UI", 10f);
                f.FormBorderStyle = FormBorderStyle.FixedDialog;
                f.StartPosition = FormStartPosition.CenterParent;
                f.MinimizeBox = f.MaximizeBox = false;
                f.AutoSize = true;
                f.AutoSizeMode = AutoSizeMode.GrowAndShrink;
                var layout = new TableLayoutPanel { AutoSize = true, ColumnCount = 1, Padding = new Padding(12), Dock = DockStyle.Fill };
                var lbl = new Label { Text = prompt, AutoSize = true };
                var txt = new TextBox { Text = value ?? "", Width = 460 };
                var ok = new Button { Text = "OK", DialogResult = DialogResult.OK, AutoSize = true };
                var cancel = new Button { Text = "Cancel", DialogResult = DialogResult.Cancel, AutoSize = true };
                var buttons = new FlowLayoutPanel { AutoSize = true, FlowDirection = FlowDirection.RightToLeft, Dock = DockStyle.Fill };
                buttons.Controls.Add(cancel);
                buttons.Controls.Add(ok);
                layout.Controls.Add(lbl);
                layout.Controls.Add(txt);
                layout.Controls.Add(buttons);
                f.Controls.Add(layout);
                f.AcceptButton = ok;
                f.CancelButton = cancel;
                return f.ShowDialog(owner) == DialogResult.OK ? txt.Text.Trim() : null;
            }
        }
    }
}
