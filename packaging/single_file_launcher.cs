// The single-file VGCS.exe. See packaging/README.md.
//
// This small program carries the whole VGCS folder build inside itself, as a
// zip resource. At the first start of a new version it unpacks that folder
// once, to %LOCALAPPDATA%\VGCS\app\<version>, then runs VGCS from there. Later
// starts only check that the unpacked files are all there, so they start as
// fast as the folder build. Old versions are deleted once no VGCS uses them.
//
// packaging/build_exe.py compiles this with the C# compiler that comes with
// the .NET Framework in Windows. Nothing needs installing, to build it or to
// run it. That compiler supports C# 5, so this file uses nothing newer.

using System;
using System.Diagnostics;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;

static class SingleFileLauncher
{
    const string PayloadResource = "VGCS.payload.zip";

    // Kept open while this version runs. Windows refuses to delete an open
    // file, so a launcher of another version can tell this folder is in use.
    const string InUseLock = "launcher-in-use.lock";

    // Windows programs, VGCS included, cannot use longer paths unless long
    // paths are switched on for the whole PC. This is the folder limit (248
    // with the terminating zero), which is a little stricter than the file one.
    const int MaxPathLength = 247;

    [DllImport("kernel32.dll")]
    static extern uint GetConsoleProcessList(uint[] processList, uint processCount);

    [STAThread]
    static int Main()
    {
        // Ctrl+C is for VGCS, which shares this console. The launcher only waits.
        Console.CancelKeyPress += delegate(object sender, ConsoleCancelEventArgs e) { e.Cancel = true; };

        // VGCS_UNPACK_DIR moves the unpacked copies, for example off a small C: drive.
        string root = Environment.GetEnvironmentVariable("VGCS_UNPACK_DIR");
        if (string.IsNullOrEmpty(root))
        {
            root = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "VGCS", "app");
        }
        string appDir = Path.Combine(root, BuildInfo.Id);
        FileStream inUse;
        // One launcher at a time touches the app folders, so two starts never
        // unpack the same version twice or delete a folder another one needs.
        using (Mutex folders = new Mutex(false, @"Local\VGCS-app-folders"))
        {
            try { folders.WaitOne(); }
            catch (AbandonedMutexException) { }  // a launcher died holding it; it is ours now
            try
            {
                PrepareAppFolder(root, appDir);
                inUse = new FileStream(
                    Path.Combine(appDir, InUseLock), FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.ReadWrite);
                RemoveOtherVersions(root, BuildInfo.Id);
            }
            catch (Exception ex)
            {
                return Fail("VGCS could not unpack itself to\n" + appDir + "\n\n" + ex.Message);
            }
            finally
            {
                folders.ReleaseMutex();
            }
        }
        int code = RunVgcs(Path.Combine(appDir, "VGCS.exe"));
        GC.KeepAlive(inUse);
        return code;
    }

    static void PrepareAppFolder(string root, string appDir)
    {
        using (Stream payload = Assembly.GetExecutingAssembly().GetManifestResourceStream(PayloadResource))
        {
            if (payload == null)
            {
                throw new InvalidOperationException("This VGCS.exe does not contain the program (a broken build).");
            }
            using (ZipArchive zip = new ZipArchive(payload, ZipArchiveMode.Read))
            {
                CheckPathLengths(zip, root);
                Directory.CreateDirectory(root);
                if (!IsComplete(zip, appDir))
                {
                    Unpack(zip, root, appDir);
                }
            }
        }
    }

    // Checks before writing anything, so a too deep folder gets a clear
    // message instead of a half-unpacked copy. Plain string lengths only: the
    // .NET path functions themselves fail on paths that are too long.
    static void CheckPathLengths(ZipArchive zip, string root)
    {
        string fullRoot;
        try
        {
            fullRoot = Path.GetFullPath(root);
        }
        catch (PathTooLongException)
        {
            fullRoot = root;
        }
        string longest = "";
        foreach (ZipArchiveEntry entry in zip.Entries)
        {
            if (entry.FullName.Length > longest.Length)
            {
                longest = entry.FullName;
            }
        }
        // The unpack goes to "<root>\<version>.partial\<entry>" before it is renamed.
        int needed = fullRoot.TrimEnd('\\', '/').Length + 1 + BuildInfo.Id.Length + ".partial".Length + 1 + longest.Length;
        if (needed > MaxPathLength)
        {
            throw new PathTooLongException(string.Format(
                "That folder's path is too long for Windows. The longest file would need {0} characters, " +
                "and the limit is {1}.\n\nSet the environment variable VGCS_UNPACK_DIR to a short folder, " +
                "for example C:\\VGCS, and start VGCS again.",
                needed, MaxPathLength));
        }
    }

    // Every file is there with the right size. This takes milliseconds, and it
    // also repairs a folder that later lost a file.
    static bool IsComplete(ZipArchive zip, string appDir)
    {
        if (!Directory.Exists(appDir))
        {
            return false;
        }
        foreach (ZipArchiveEntry entry in zip.Entries)
        {
            if (entry.FullName.EndsWith("/"))
            {
                continue;
            }
            FileInfo file = new FileInfo(Path.Combine(appDir, entry.FullName));
            if (!file.Exists || file.Length != entry.Length)
            {
                return false;
            }
        }
        return true;
    }

    static void Unpack(ZipArchive zip, string root, string appDir)
    {
        long total = 0;
        foreach (ZipArchiveEntry entry in zip.Entries)
        {
            total += entry.Length;
        }
        long free = new DriveInfo(Path.GetPathRoot(root)).AvailableFreeSpace;
        long needed = total + (100L << 20);
        if (free < needed)
        {
            throw new IOException(string.Format(
                "Not enough free disk space. VGCS needs {0} MB there, and {1} MB is free.",
                needed >> 20, free >> 20));
        }
        if (Directory.Exists(appDir))
        {
            Console.WriteLine("VGCS: some unpacked files are missing or damaged. Unpacking again, to " + appDir);
        }
        else
        {
            Console.WriteLine("VGCS: first start of this version. Unpacking it once, to " + appDir);
        }

        // Unpack beside the final folder, then rename: a half-unpacked copy is
        // never mistaken for a complete one.
        string partial = appDir + ".partial";
        if (Directory.Exists(partial))
        {
            Directory.Delete(partial, true);
        }
        string partialRoot = Path.GetFullPath(partial) + Path.DirectorySeparatorChar;
        long done = 0;
        int shown = -1;
        foreach (ZipArchiveEntry entry in zip.Entries)
        {
            string target = Path.GetFullPath(Path.Combine(partial, entry.FullName));
            if (!target.StartsWith(partialRoot, StringComparison.OrdinalIgnoreCase))
            {
                throw new InvalidDataException("Unexpected path in the program data: " + entry.FullName);
            }
            if (entry.FullName.EndsWith("/"))
            {
                Directory.CreateDirectory(target);
                continue;
            }
            Directory.CreateDirectory(Path.GetDirectoryName(target));
            entry.ExtractToFile(target, true);
            done += entry.Length;
            int percent = (int)(done * 100 / Math.Max(1L, total));
            if (percent != shown)
            {
                Console.Write("\rVGCS: unpacking {0,3}%", percent);
                shown = percent;
            }
        }
        Console.WriteLine();
        if (Directory.Exists(appDir))
        {
            // An earlier copy of this version lost files: replace it.
            string broken = appDir + ".broken";
            if (Directory.Exists(broken))
            {
                Directory.Delete(broken, true);
            }
            Directory.Move(appDir, broken);
            TryDelete(broken);
        }
        Directory.Move(partial, appDir);
    }

    // Deletes the folders of other versions, and leftovers of interrupted
    // unpacks, unless a VGCS is running from them.
    static void RemoveOtherVersions(string root, string current)
    {
        foreach (string dir in Directory.GetDirectories(root))
        {
            if (string.Equals(Path.GetFileName(dir), current, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }
            if (IsRunningFrom(dir))
            {
                continue;
            }
            try
            {
                File.Delete(Path.Combine(dir, InUseLock));
            }
            catch (IOException)
            {
                continue;  // its launcher holds the lock: in use
            }
            catch (UnauthorizedAccessException)
            {
                continue;
            }
            TryDelete(dir);
        }
    }

    static bool IsRunningFrom(string dir)
    {
        string prefix = Path.GetFullPath(dir).TrimEnd('\\') + "\\";
        foreach (Process process in Process.GetProcessesByName("VGCS"))
        {
            try
            {
                if (process.MainModule.FileName.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
                {
                    return true;
                }
            }
            catch (Exception)
            {
                // Another user's process, or one that just exited.
            }
            finally
            {
                process.Dispose();
            }
        }
        return false;
    }

    static void TryDelete(string dir)
    {
        try
        {
            Directory.Delete(dir, true);
        }
        catch (Exception)
        {
            // Something still holds a file. The next start tries again.
        }
    }

    static int RunVgcs(string exe)
    {
        ProcessStartInfo start = new ProcessStartInfo(exe, ArgumentsAfterProgramName());
        start.UseShellExecute = false;  // share this console, so the VGCS log shows here
        using (Process vgcs = Process.Start(start))
        {
            vgcs.WaitForExit();
            return vgcs.ExitCode;
        }
    }

    // The command line after the program name, passed on exactly as typed.
    static string ArgumentsAfterProgramName()
    {
        string line = Environment.CommandLine;
        int i = 0;
        if (line.StartsWith("\""))
        {
            int end = line.IndexOf('"', 1);
            i = end < 0 ? line.Length : end + 1;
        }
        else
        {
            while (i < line.Length && !char.IsWhiteSpace(line[i]))
            {
                i++;
            }
        }
        return line.Substring(i).TrimStart();
    }

    static int Fail(string message)
    {
        Console.Error.WriteLine("VGCS: " + message.Replace("\n\n", " ").Replace("\n", " "));
        // A double-clicked VGCS.exe has a console of its own, which closes at
        // once, so show the message in a window too. Started from a terminal,
        // the console is shared and the message stays readable there.
        if (GetConsoleProcessList(new uint[2], 2) == 1)
        {
            MessageBox.Show(message, "VGCS", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
        return 1;
    }
}
