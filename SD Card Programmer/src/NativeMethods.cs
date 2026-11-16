using System;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;

namespace SdCardProgrammer
{
    internal static class NativeMethods
    {
        public const uint GENERIC_READ = 0x80000000;
        public const uint GENERIC_WRITE = 0x40000000;
        public const uint FILE_SHARE_READ = 0x1;
        public const uint FILE_SHARE_WRITE = 0x2;
        public const uint OPEN_EXISTING = 3;
        public const uint FILE_FLAG_NO_BUFFERING = 0x20000000;
        public const uint FILE_FLAG_WRITE_THROUGH = 0x80000000;

        // Device I/O control codes
        public const uint IOCTL_STORAGE_GET_DEVICE_NUMBER = 0x2D1080;
        public const uint IOCTL_STORAGE_QUERY_PROPERTY = 0x2D1400;
        public const uint IOCTL_STORAGE_CHECK_VERIFY2 = 0x2D0800;
        public const uint IOCTL_STORAGE_MEDIA_REMOVAL = 0x2D4804;
        public const uint IOCTL_STORAGE_EJECT_MEDIA = 0x2D4808;
        public const uint IOCTL_DISK_GET_LENGTH_INFO = 0x7405C;
        public const uint IOCTL_DISK_GET_DRIVE_GEOMETRY_EX = 0x700A0;
        public const uint IOCTL_DISK_UPDATE_PROPERTIES = 0x70140;
        public const uint IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS = 0x560000;
        public const uint FSCTL_LOCK_VOLUME = 0x90018;
        public const uint FSCTL_UNLOCK_VOLUME = 0x9001C;
        public const uint FSCTL_DISMOUNT_VOLUME = 0x90020;

        public const uint FILE_DEVICE_DISK = 7;

        public const uint MEM_COMMIT = 0x1000;
        public const uint MEM_RESERVE = 0x2000;
        public const uint MEM_RELEASE = 0x8000;
        public const uint PAGE_READWRITE = 0x04;

        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        public static extern SafeFileHandle CreateFile(string fileName, uint desiredAccess, uint shareMode,
            IntPtr securityAttributes, uint creationDisposition, uint flagsAndAttributes, IntPtr templateFile);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool DeviceIoControl(SafeFileHandle device, uint ioControlCode,
            byte[] inBuffer, int inBufferSize, byte[] outBuffer, int outBufferSize,
            out int bytesReturned, IntPtr overlapped);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool WriteFile(SafeFileHandle file, IntPtr buffer, int numberOfBytesToWrite,
            out int numberOfBytesWritten, IntPtr overlapped);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool ReadFile(SafeFileHandle file, IntPtr buffer, int numberOfBytesToRead,
            out int numberOfBytesRead, IntPtr overlapped);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool SetFilePointerEx(SafeFileHandle file, long distanceToMove,
            out long newFilePointer, uint moveMethod);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool FlushFileBuffers(SafeFileHandle file);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern IntPtr VirtualAlloc(IntPtr address, UIntPtr size, uint allocationType, uint protect);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool VirtualFree(IntPtr address, UIntPtr size, uint freeType);

        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        public static extern IntPtr FindFirstVolume(StringBuilder volumeName, int bufferLength);

        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        public static extern bool FindNextVolume(IntPtr findVolume, StringBuilder volumeName, int bufferLength);

        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool FindVolumeClose(IntPtr findVolume);

        [DllImport("kernel32.dll")]
        public static extern uint GetLogicalDrives();

        [DllImport("kernel32.dll")]
        public static extern uint SetErrorMode(uint mode);

        public static readonly IntPtr INVALID_HANDLE_VALUE = new IntPtr(-1);
    }
}
