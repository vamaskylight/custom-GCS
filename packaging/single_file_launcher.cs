// The single-file VGCS.exe. See packaging/README.md.
//
// This small program carries the whole VGCS folder build inside itself, as a
// zip resource. At the first start of a new version it unpacks that folder
// once, to %LOCALAPPDATA%\VGCS\app\<version>, then runs VGCS from there. Later
// starts only check that the unpacked files are all there, so they start as
// fast as the folder build. Old versions are deleted once no VGCS uses them.
//
// It shows no console window. VGCS runs with a hidden console, which FFmpeg
// and the tracker workers share, so none of them opens a window either.
// What VGCS prints goes to Documents\VGCS\logs\VGCS-<date>_<time>.log. When a
// script or a test starts VGCS.exe and reads its output, the output goes to
// that reader instead, and no window or message box ever waits for a click.
//
// packaging/build_exe.py compiles this with the C# compiler that comes with
// the .NET Framework in Windows. Nothing needs installing, to build it or to
// run it. That compiler supports C# 5, so this file uses nothing newer.

using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
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

    const int LogsToKeep = 30;

    [DllImport("kernel32.dll")]
    static extern IntPtr GetStdHandle(int stdHandle);

    [DllImport("kernel32.dll")]
    static extern uint GetFileType(IntPtr file);

    [STAThread]
    static int Main()
    {
        Application.EnableVisualStyles();
        bool captured = OutputIsCaptured();

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
                PrepareAppFolder(root, appDir, captured);
                inUse = new FileStream(
                    Path.Combine(appDir, InUseLock), FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.ReadWrite);
                RemoveOtherVersions(root, BuildInfo.Id);
            }
            catch (Exception ex)
            {
                return Fail("VGCS could not unpack itself to\n" + appDir + "\n\n" + ex.Message, captured);
            }
            finally
            {
                folders.ReleaseMutex();
            }
        }
        int code = RunVgcs(Path.Combine(appDir, "VGCS.exe"), captured);
        GC.KeepAlive(inUse);
        return code;
    }

    // True when whoever started VGCS.exe reads its output: a script, a test,
    // or "VGCS.exe > file". A double-click gives it no output at all.
    static bool OutputIsCaptured()
    {
        uint type = GetFileType(GetStdHandle(-11));  // STD_OUTPUT_HANDLE
        return type == 1 || type == 3;  // FILE_TYPE_DISK or FILE_TYPE_PIPE
    }

    static void PrepareAppFolder(string root, string appDir, bool captured)
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
                if (IsComplete(zip, appDir))
                {
                    return;
                }
                bool repair = Directory.Exists(appDir);
                Console.WriteLine(repair
                    ? "VGCS: some unpacked files are missing or damaged. Unpacking again, to " + appDir
                    : "VGCS: first start of this version. Unpacking it once, to " + appDir);
                if (captured)
                {
                    Unpack(zip, root, appDir, delegate(int percent) { Console.Write("\rVGCS: unpacking {0,3}%", percent); });
                    Console.WriteLine();
                }
                else
                {
                    UnpackShowingProgress(zip, root, appDir, repair);
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

    // The first start takes about half a minute. With no console, show that
    // something is happening, or people start VGCS.exe again and again.
    static void UnpackShowingProgress(ZipArchive zip, string root, string appDir, bool repair)
    {
        Exception failure = null;
        Thread worker = null;
        using (Form window = new Form())
        {
            window.Text = "VGCS";
            window.FormBorderStyle = FormBorderStyle.FixedDialog;
            window.ControlBox = false;  // it closes by itself when the unpack is done
            window.StartPosition = FormStartPosition.CenterScreen;
            window.ClientSize = new Size(440, 96);
            try
            {
                window.Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath);
            }
            catch (Exception)
            {
                // No icon is fine.
            }
            Label text = new Label();
            text.SetBounds(14, 12, 412, 40);
            text.Text = repair
                ? "Some VGCS files were missing. Unpacking them again."
                : "Preparing VGCS for its first start.\nThis happens once for each new version.";
            ProgressBar bar = new ProgressBar();
            bar.SetBounds(14, 60, 412, 22);
            window.Controls.Add(text);
            window.Controls.Add(bar);
            window.Shown += delegate
            {
                worker = new Thread(delegate()
                {
                    try
                    {
                        Unpack(zip, root, appDir, delegate(int percent)
                        {
                            window.BeginInvoke((MethodInvoker)delegate { bar.Value = percent; });
                        });
                    }
                    catch (Exception ex)
                    {
                        failure = ex;
                    }
                    window.BeginInvoke((MethodInvoker)delegate { window.Close(); });
                });
                worker.IsBackground = true;
                worker.Start();
            };
            Application.Run(window);
        }
        if (worker != null)
        {
            worker.Join();
        }
        if (failure != null)
        {
            throw failure;
        }
    }

    static void Unpack(ZipArchive zip, string root, string appDir, Action<int> progress)
    {
        long total = 0;
        foreach (ZipArchiveEntry entry in zip.Entries)
        {
            total += entry.Length;
        }
        long free = new DriveInfo(Path.GetPathRoot(Path.GetFullPath(root))).AvailableFreeSpace;
        long needed = total + (100L << 20);
        if (free < needed)
        {
            throw new IOException(string.Format(
                "Not enough free disk space. VGCS needs {0} MB there, and {1} MB is free.",
                needed >> 20, free >> 20));
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
                progress(percent);
                shown = percent;
            }
        }
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

    static int RunVgcs(string exe, bool captured)
    {
        string logPath = null;
        TextWriter output;
        TextWriter errors;
        if (captured)
        {
            output = Writer(Console.OpenStandardOutput());
            errors = Writer(Console.OpenStandardError());
        }
        else
        {
            output = OpenLogFile(out logPath);
            errors = output;
        }
        ProcessStartInfo start = new ProcessStartInfo(exe, ArgumentsAfterProgramName());
        start.UseShellExecute = false;
        // A hidden console: VGCS, FFmpeg and the workers share it, so none of
        // them shows a console window.
        start.CreateNoWindow = true;
        start.RedirectStandardOutput = true;
        start.RedirectStandardError = true;
        start.StandardOutputEncoding = Encoding.UTF8;
        start.StandardErrorEncoding = Encoding.UTF8;
        object gate = new object();
        int code;
        try
        {
            using (Process vgcs = new Process())
            {
                vgcs.StartInfo = start;
                vgcs.OutputDataReceived += delegate(object sender, DataReceivedEventArgs e)
                {
                    if (e.Data != null)
                    {
                        lock (gate) { output.WriteLine(e.Data); }
                    }
                };
                vgcs.ErrorDataReceived += delegate(object sender, DataReceivedEventArgs e)
                {
                    if (e.Data != null)
                    {
                        lock (gate) { errors.WriteLine(e.Data); }
                    }
                };
                vgcs.Start();
                vgcs.BeginOutputReadLine();
                vgcs.BeginErrorReadLine();
                vgcs.WaitForExit();  // without a timeout this also waits until all output is read
                code = vgcs.ExitCode;
            }
        }
        catch (Exception ex)
        {
            // For example an antivirus program that blocks the unpacked VGCS.exe.
            return Fail("VGCS could not start\n" + exe + "\n\n" + ex.Message, captured);
        }
        lock (gate)
        {
            output.Flush();
            errors.Flush();
        }
        if (!captured)
        {
            output.Dispose();
            ReportOutcome(code, logPath);
        }
        return code;
    }

    static TextWriter Writer(Stream stream)
    {
        StreamWriter writer = new StreamWriter(stream, new UTF8Encoding(false));
        writer.AutoFlush = true;
        return writer;
    }

    // Documents\VGCS\logs, beside VGCS's other logs. LOCALAPPDATA when
    // Documents is not writable. VGCS_LOG_DIR overrides both.
    static TextWriter OpenLogFile(out string path)
    {
        string[] places =
        {
            Environment.GetEnvironmentVariable("VGCS_LOG_DIR"),
            LogsUnder(Environment.SpecialFolder.MyDocuments),
            LogsUnder(Environment.SpecialFolder.LocalApplicationData),
        };
        foreach (string folder in places)
        {
            if (string.IsNullOrEmpty(folder))
            {
                continue;
            }
            try
            {
                Directory.CreateDirectory(folder);
                RemoveOldLogs(folder);
                path = Path.Combine(folder, "VGCS-" + DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".log");
                StreamWriter writer = new StreamWriter(path, true, new UTF8Encoding(false));
                writer.AutoFlush = true;  // the log stays complete even if VGCS.exe is killed
                return writer;
            }
            catch (Exception)
            {
                // Try the next place.
            }
        }
        path = null;
        return TextWriter.Null;
    }

    static string LogsUnder(Environment.SpecialFolder folder)
    {
        string basePath = Environment.GetFolderPath(folder);
        return string.IsNullOrEmpty(basePath) ? null : Path.Combine(basePath, "VGCS", "logs");
    }

    // Keeps the newest logs; the new one makes it LogsToKeep again.
    static void RemoveOldLogs(string folder)
    {
        string[] logs = Directory.GetFiles(folder, "VGCS-*.log");
        Array.Sort(logs, StringComparer.OrdinalIgnoreCase);  // the names sort by date and time
        for (int i = 0; i < logs.Length - (LogsToKeep - 1); i++)
        {
            try
            {
                File.Delete(logs[i]);
            }
            catch (Exception)
            {
                // In use by another VGCS: keep it.
            }
        }
    }

    // With no console, VGCS that stopped with an error would just vanish.
    static void ReportOutcome(int code, string logPath)
    {
        string where = logPath == null ? "" : "\n\nThe log is in\n" + logPath;
        if (Array.IndexOf(Environment.GetCommandLineArgs(), "--selfcheck") > 0)
        {
            string result = code == 0 ? "All checks passed." : code + " check(s) failed.";
            MessageBox.Show(result + where, "VGCS self-check", MessageBoxButtons.OK,
                code == 0 ? MessageBoxIcon.Information : MessageBoxIcon.Warning);
        }
        else if (code != 0)
        {
            MessageBox.Show("VGCS stopped with an error (code " + code + ")." + where +
                "\n\nPlease send this log when you report the problem.", "VGCS",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
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

    static int Fail(string message, bool captured)
    {
        Console.Error.WriteLine("VGCS: " + message.Replace("\n\n", " ").Replace("\n", " "));
        if (!captured)
        {
            MessageBox.Show(message, "VGCS", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
        return 1;
    }
}
