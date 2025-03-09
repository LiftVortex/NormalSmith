using System;
using System.Diagnostics;
using System.IO;
using System.IO.Compression;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows;

namespace UpdaterApp
{
    public partial class MainWindow : Window
    {

        public MainWindow()
        {
            InitializeComponent();
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // Check for a command-line argument for the install directory.
            string[] args = Environment.GetCommandLineArgs();
            if (args.Length < 2)
            {
                MessageBox.Show(
                    "Please use the 'Check for Updates' button in the main NormalSmith application.",
                    "Updater",
                    MessageBoxButton.OK,
                    MessageBoxImage.Information);
                Application.Current.Shutdown();
                return;
            }

            // Use the provided install directory.
            string installDir = args[1];

            await RunUpdaterAsync(installDir);
        }


        private async Task RunUpdaterAsync(string installDir)
        {
            try
            {
                // [Step 0] Initial information
                UpdateUI("Starting Updater...", 0);
                Console.WriteLine("============================");
                Console.WriteLine(" NormalSmith Updater Script");
                Console.WriteLine("============================");
                Console.WriteLine($"Script is running from: {installDir}");
                Console.WriteLine($"InstallDir (destination): {installDir}");

                // --- Step 1: Set Download URL ---
                string downloadUrl = "https://github.com/LiftVortex/NormalSmith/releases/latest/download/NormalSmith-Build.zip";

                // --- Step 2: Define temporary paths ---
                UpdateUI("Step 2: Setting up temporary paths...", 20);
                string tempZip = Path.Combine(Path.GetTempPath(), "NormalSmith-Build.zip");
                string tempExtractDir = Path.Combine(Path.GetTempPath(), "NormalSmith_Update");

                Console.WriteLine("[Step 2] Defining temporary file/folder paths...");
                Console.WriteLine($"   > Initial temp extract directory: {tempExtractDir}");

                // Remove trailing directory separator if any
                tempExtractDir = tempExtractDir.TrimEnd(Path.DirectorySeparatorChar);

                // Ensure a clean extraction directory
                if (Directory.Exists(tempExtractDir))
                {
                    Console.WriteLine($"   > Removing existing folder: {tempExtractDir}");
                    Directory.Delete(tempExtractDir, true);
                }

                Console.WriteLine($"   > Creating directory: {tempExtractDir}");
                Directory.CreateDirectory(tempExtractDir);
                tempExtractDir = new DirectoryInfo(tempExtractDir).FullName;
                Console.WriteLine($"   > Normalized long path: {tempExtractDir}");
                Console.WriteLine($"   > ZIP file will be downloaded to: {tempZip}");

                await Task.Delay(1000);

                // --- Step 3: Download the update zip ---
                UpdateUI("Step 3: Downloading update...", 40);
                Console.WriteLine($"[Step 3] Downloading update from: {downloadUrl} ...");

                // Show the download progress bar and label
                Dispatcher.Invoke(() =>
                {
                    downloadProgressBar.Visibility = Visibility.Visible;
                    downloadProgressLabel.Visibility = Visibility.Visible;
                    downloadProgressBar.Value = 0;
                });

                using (HttpClient client = new HttpClient())
                {
                    // Request the content with response headers only first
                    using (var response = await client.GetAsync(downloadUrl, HttpCompletionOption.ResponseHeadersRead))
                    {
                        response.EnsureSuccessStatusCode();
                        var totalBytes = response.Content.Headers.ContentLength ?? -1L;
                        using (var contentStream = await response.Content.ReadAsStreamAsync())
                        using (var fileStream = new FileStream(tempZip, FileMode.Create, FileAccess.Write, FileShare.None, 8192, true))
                        {
                            var buffer = new byte[8192];
                            long totalRead = 0;
                            int bytesRead;
                            while ((bytesRead = await contentStream.ReadAsync(buffer, 0, buffer.Length)) > 0)
                            {
                                await fileStream.WriteAsync(buffer, 0, bytesRead);
                                totalRead += bytesRead;
                                if (totalBytes != -1)
                                {
                                    // Calculate the progress percentage.
                                    double progress = (double)totalRead / totalBytes * 100;
                                    // Update the download progress bar on the UI thread.
                                    Dispatcher.Invoke(() =>
                                    {
                                        downloadProgressBar.Value = progress;
                                    });
                                }
                            }
                        }
                    }
                }

                // Hide the download progress bar and label after download completes.
                Dispatcher.Invoke(() =>
                {
                    downloadProgressBar.Visibility = Visibility.Collapsed;
                    downloadProgressLabel.Visibility = Visibility.Collapsed;
                });

                Console.WriteLine("   > Download successful!");
                Console.WriteLine("   > Pausing briefly...");


                // --- Step 4: Extract the zip ---
                UpdateUI("Step 4: Extracting update package...", 60);
                Console.WriteLine($"[Step 4] Extracting ZIP to temporary directory: {tempExtractDir}");
                ZipFile.ExtractToDirectory(tempZip, tempExtractDir);
                Console.WriteLine("   > Extraction complete!");

                // --- Step 5: Copy updated files to installation folder ---
                UpdateUI("Step 5: Copying updated files...", 80);
                Console.WriteLine($"[Step 5] Copying updated files to: {installDir}");
                CopyDirectory(tempExtractDir, installDir);

                // --- Step 6: Clean up temporary files ---
                UpdateUI("Step 6: Cleaning up...", 90);
                Console.WriteLine("[Step 6] Removing temporary files...");
                if (File.Exists(tempZip))
                {
                    File.Delete(tempZip);
                    Console.WriteLine($"   > Removed ZIP file: {tempZip}");
                }
                if (Directory.Exists(tempExtractDir))
                {
                    Directory.Delete(tempExtractDir, true);
                    Console.WriteLine($"   > Removed temp extract directory: {tempExtractDir}");
                }

                // --- Step 7: Launch the updated application ---
                UpdateUI("Step 7: Launching application...", 100);
                Console.WriteLine("[Step 7] Launching the updated NormalSmith application...");
                string exePath = Path.Combine(installDir, "NormalSmith.exe");
                Console.WriteLine($"   > Executable Path: {exePath}");

                UpdateUI("Update Process Completed", 100);
                await Task.Delay(1000);

                if (File.Exists(exePath))
                {
                    Process.Start(new ProcessStartInfo(exePath) { UseShellExecute = true });
                    Console.WriteLine("   > Application launched.");
                }
                else
                {
                    Console.WriteLine($"   > ERROR: Could not find NormalSmith.exe at: {exePath}");
                }

                Console.WriteLine("============================");
                Console.WriteLine(" Update Process Completed  ");
                Console.WriteLine("============================");

                this.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"An error occurred: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        /// <summary>
        /// Updates the status text and progress bar.
        /// </summary>
        private void UpdateUI(string status, int progress)
        {
            // Ensure UI updates occur on the UI thread.
            Dispatcher.Invoke(() =>
            {
                txtStatus.Text = status;
                progressBar.Value = progress;
            });
        }

        /// <summary>
        /// Recursively copies all files and directories from source to destination.
        /// </summary>
        public static void CopyDirectory(string sourceDir, string targetDir)
        {
            // Create target directory if it doesn't exist.
            Directory.CreateDirectory(targetDir);

            foreach (string file in Directory.GetFiles(sourceDir))
            {
                string targetFilePath = Path.Combine(targetDir, Path.GetFileName(file));
                File.Copy(file, targetFilePath, true);
            }

            foreach (string directory in Directory.GetDirectories(sourceDir))
            {
                string targetDirectoryPath = Path.Combine(targetDir, Path.GetFileName(directory));
                CopyDirectory(directory, targetDirectoryPath);
            }
        }
    }

    // Class to hold configuration properties
    public class UpdaterConfig
    {
        public string downloadUrl { get; set; }
    }
}
