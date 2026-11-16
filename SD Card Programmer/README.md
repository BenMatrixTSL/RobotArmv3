# SD Card Programmer

A Windows tool for programming SD cards in batches: clone a Raspberry Pi `.img` onto the cards,
or copy the contents of a `.zip` onto them. Every ticked card is written **at the same time**, so a hub full of readers programs a whole batch in about the time one card takes. When a batch finishes,
the status panel turns **green** and the cards are safely ejected. Press **PROGRAM NEXT BATCH**
(or turn on auto-start) to do it again.

## Running it

Copy this folder to the PC and double-click `SDCardProgrammer.exe`. Windows asks for administrator
rights (UAC) because writing to SD cards needs raw disk access.

- Requires Windows 10 or 11 (it uses .NET Framework 4.8, which is built in). Nothing else to install.
- Optional: [7-Zip](https://www.7-zip.org) lets you use `.img.xz` / `.7z` images directly. Install it,
  or put `7z.exe` + `7z.dll` in a `tools\` folder next to the exe.

## Using it

1. **Card ports**: tick every reader slot or USB card reader to program this round. Each port shows its
   drive letter, whether a card is in, and the card size. The ticked ports are remembered by drive letter.
2. **Job type**
   - *Clone disk image*: writes the whole image to the card, sector by sector, like Win32DiskImager or
     Raspberry Pi Imager. Accepts `.img`, `.img.gz`, a `.zip` that contains an `.img`, and `.img.xz`/`.7z`
     (needs 7-Zip).
   - *Copy ZIP contents*: optionally formats the card (FAT32 / exFAT / NTFS, with a label), then extracts
     every file in the ZIP onto it. *Remove single top folder* handles ZIPs where everything sits inside
     one folder.
3. **Source file**: Browse, or pick one of the recently used files from the drop-down.
4. Press **START**. You're asked to confirm the list of cards that will be erased, then all the cards are
   programmed **at the same time**. Each port line shows its own phase, % and MB/s. The big panel shows the
   overall %, total throughput and time left for the slowest card. Each card is ejected as soon as it's
   finished, and its line shows ✔ DONE or ✖ FAILED. One card failing doesn't affect the others.
5. **Big status panel**
   - Grey: ready
   - Amber: working (shows the % done, speed and time left in the line underneath)
   - **Green: COMPLETE**, the cards are ejected and safe to remove
   - Red: something failed (the reason is shown, and the full detail is in the log)
   - Blue: waiting for the next cards
6. **PROGRAM NEXT BATCH**: swap the cards. Programming starts once every ticked port holds a new card. The
   app notices when a finished card is removed, so it never re-programs the card you just did. You're
   only asked to confirm again if the set of ports changes. Tick *Auto-start* to go fully hands-free:
   swap the cards and the next batch starts by itself.

### Saved jobs (friendly names) - one shared list for everyone

Set up a job (type, file, options), press **Save as...** and give it a name such as
"Robot Arm Pi image v3". Pick it from **Saved job** to load everything at once.

Saved jobs are stored in **`jobs.xml` next to `SDCardProgrammer.exe`**, so the whole team can share one list:

1. **Build the list:** copy this folder somewhere you can write to, run the exe and save your jobs. They
   go into `jobs.xml` in that folder.
2. **Publish it:** copy the folder, including `jobs.xml`, to a read-only network share. Anyone who runs
   the exe from there gets the same job list. On a read-only share, Save and Delete are greyed out.
3. **Update it:** edit `jobs.xml` from a PC that can write to the share (run the exe there, or edit
   the XML by hand). Everyone else sees the change the next time they open the Saved job drop-down.

Where the job's files should live:

- **Inside the tool's folder** (e.g. an `images\` subfolder on the share). These are stored as relative
  paths (`images\robot-arm-v3.img`), so they keep working wherever the folder is copied. This is the
  simplest option.
- **Anywhere else on the network:** use a full `\\server\share\...` path, *not* a mapped drive letter
  like `Z:`. The tool runs as administrator, and Windows hides mapped drive letters from
  administrator programs. The app warns you if you save a shared job that points at a file on the
  local PC.
- Also start the tool itself from a `\\server\share\...` path or a shortcut to one, for the same reason.

If the exe's folder is read-only *and* has no `jobs.xml`, the app falls back to a personal list in
`%APPDATA%\SD Card Programmer\jobs.xml`. The log shows which job list is in use when the app starts.
### Speed and safety features

- **Write all ticked cards at the same time** (on by default): best with one reader per port of a USB 3
  hub, where each card's own write speed is the limit. Untick it to program the cards one after another,
  for example if a cheap hub or reader misbehaves under load. In ZIP mode the quick format step runs
  one card at a time (diskpart can't run in parallel), and the file copying then runs in parallel.

- **Skip unused space** (image mode): only writes up to the end of the last partition in the image. This
  speeds up images that have empty space at the end, such as a freshly downloaded Pi OS image or a shrunk
  image. It also lets them fit onto smaller cards. An image read from a card whose root partition was
  expanded to fill the card still has to be written in full, and needs a card at least as large.
- **Verify**: reads the card back and checks its MD5 against what was written. This catches fake or
  faulty cards.
- The partition table (sector 0) is written last, so a cancelled or failed write never leaves a
  half-valid card that looks bootable.
- Only USB, SD and MMC removable devices are listed. The disk that holds Windows is never offered or
  written. Devices larger than 256 GB need an extra confirmation.

## Files

- `jobs.xml` (next to the exe): the shared saved-job list
- `settings.xml`: each user's recent files, ticked ports and checkbox choices, in `%APPDATA%\SD Card Programmer\`
- `log.txt`: a log of every job, in the same folder

## Building

Run `build.bat`. It uses the C# compiler that ships with Windows
(`%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\csc.exe`), so no Visual Studio is needed. The source
is in `src\`:

| File | Purpose |
|---|---|
| `MainForm.cs` | The window, batch queue, card-swap detection, saved jobs |
| `Jobs.cs` | Image writing and verifying, ZIP extraction, MBR parsing |
| `DiskTools.cs` | Listing disks, safety checks, lock/dismount, eject, diskpart formatting |
| `JobStore.cs` | The shared `jobs.xml` job list (location, read-only detection, relative paths) |
| `Settings.cs` | Per-user preferences (XML) |
| `NativeMethods.cs` | Win32 calls |
