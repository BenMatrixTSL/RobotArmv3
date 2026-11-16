using System;
using System.Security.Principal;
using System.Windows.Forms;

namespace SdCardProgrammer
{
    static class Program
    {
        [STAThread]
        static void Main()
        {
            // Don't let Windows pop up "insert a disk" dialogs when we probe empty card readers.
            NativeMethods.SetErrorMode(0x0001 | 0x8000); // SEM_FAILCRITICALERRORS | SEM_NOOPENFILEERRORBOX

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            var principal = new WindowsPrincipal(WindowsIdentity.GetCurrent());
            bool skipAdminCheck = false;
#if UITEST
            skipAdminCheck = true;   // layout testing only
#endif
            if (!skipAdminCheck && !principal.IsInRole(WindowsBuiltInRole.Administrator))
            {
                MessageBox.Show("SD Card Programmer needs administrator rights to write to SD cards.\n\n" +
                                "Right-click it and choose 'Run as administrator'.",
                                "SD Card Programmer", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            Application.ThreadException += (s, e) =>
                MessageBox.Show(e.Exception.ToString(), "Unexpected error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            Application.Run(new MainForm());
        }
    }
}
