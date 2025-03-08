using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing; // For System.Drawing.Bitmap and System.Drawing.Color
using System.Drawing.Imaging; // For System.Drawing.Imaging.PixelFormat
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Assimp;
using NormalSmith.Engine;
using NormalSmith.HelperFunctions;
using MediaColor = System.Windows.Media.Color;
using MediaColorConverter = System.Windows.Media.ColorConverter;
using WinForms = System.Windows.Forms;

namespace NormalSmith
{
    #region Assimp Extensions
    /// <summary>
    /// Provides extension methods for Assimp types.
    /// </summary>
    public static class AssimpExtensions
    {
        /// <summary>
        /// Converts an Assimp.Matrix4x4 to a System.Numerics.Matrix4x4.
        /// </summary>
        public static System.Numerics.Matrix4x4 ToMatrix4x4(this Assimp.Matrix4x4 m)
        {
            return new System.Numerics.Matrix4x4(
                m.A1, m.A2, m.A3, m.A4,
                m.B1, m.B2, m.B3, m.B4,
                m.C1, m.C2, m.C3, m.C4,
                m.D1, m.D2, m.D3, m.D4
            );
        }
    }
    #endregion

    public partial class MainWindow : Window
    {
        #region Fields and Private Variables
        // Baking preview interval in milliseconds.
        private int previewInterval = 500;
        // Flag indicating whether fast preview mode is enabled.
        private bool fastPreviewEnabled = false; // Default to off (500ms interval)
        // Thread-local random generator for use in parallel processing.
        private static readonly ThreadLocal<Random> threadRandom = new ThreadLocal<Random>(() => new Random(Guid.NewGuid().GetHashCode()));

        // Model and scene data.
        private Scene loadedScene;            // Holds the loaded 3D scene.
        private int selectedMeshIndex = 0;    // Index of the currently selected mesh.
        private string modelPath = "sphere.fbx"; // Default model file path.
        private int textureWidth = 1024;       // Default texture width.
        private int textureHeight = 1024;      // Default texture height.

        // Baking settings.
        private bool useTangentSpace = true;
        private bool useCosineDistribution = true;
        private bool generateBentNormalMap = true;
        private bool generateOcclusionMap = true;
        private bool useEnhancedTangentProcessing = false;

        // Swizzle settings for normal map channel adjustment.
        private Vector3 swizzle = new Vector3(1, -1, 1);

        // Ray tracing parameters.
        private int raySampleCount = 64;
        private float maxRayDistance = 1000f;
        private float occlusionThreshold = 0.726f;
        private float rayOriginBias = 0.01f;

        // Last-used folder paths for persistence.
        private string lastModelFolder;
        private string lastAlphaFolder;

        // Final output bitmaps (used when generating both maps).
        private System.Drawing.Bitmap finalBentMap;
        private System.Drawing.Bitmap finalOccMap;

        // Cancellation token source for the baking operation.
        private CancellationTokenSource bakeCancellationTokenSource;

        // Global list of occluder triangles used for occlusion testing.
        private List<TriangleData> occluderTriangles;
        #endregion

        #region Private Types
        /// <summary>
        /// Represents a triangle (with associated UV coordinates) used in occlusion testing.
        /// </summary>
        private class TriangleData
        {
            public Vector3 v0, v1, v2;
            public Vector2 uv0, uv1, uv2;

            public TriangleData(Vector3 a, Vector3 b, Vector3 c, Vector2 uvA, Vector2 uvB, Vector2 uvC)
            {
                v0 = a; v1 = b; v2 = c;
                uv0 = uvA; uv1 = uvB; uv2 = uvC;
            }

            /// <summary>
            /// Returns the centroid of the triangle.
            /// </summary>
            public Vector3 Centroid() => (v0 + v1 + v2) / 3.0f;
        }
        #endregion

        #region Constructor
        /// <summary>
        /// Initializes the MainWindow, sets up event handlers for loading and closing.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += MainWindow_Loaded;
            this.Closing += MainWindow_Closing;
        }
        #endregion

        #region Persistent Settings
        /// <summary>
        /// Loads persistent settings and initializes the UI on window load.
        /// </summary>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // Load dark mode preference.
            bool isDarkMode = Properties.Settings.Default.IsDarkMode;
            menuDarkModeToggle.IsChecked = isDarkMode;
            menuDarkModeToggle_Click(null, null);

            // Load highlight color and update UI brushes.
            string highlightColor = Properties.Settings.Default.HighlightColor;
            if (!string.IsNullOrEmpty(highlightColor))
            {
                MediaColor newHighlight = (MediaColor)MediaColorConverter.ConvertFromString(highlightColor);
                Resources["ButtonBackgroundBrush"] = new SolidColorBrush(newHighlight);
                Resources["GroupBoxHeaderForegroundBrush"] = new SolidColorBrush(newHighlight);
                Resources["ButtonHoverBackgroundBrush"] = new SolidColorBrush(DarkenColor(newHighlight, 0.8));
                Resources["ButtonPressedBackgroundBrush"] = new SolidColorBrush(DarkenColor(newHighlight, 0.6));
            }

            // Load texture dimensions and other baking options.
            txtWidth.Text = Properties.Settings.Default.TextureWidth.ToString();
            txtHeight.Text = Properties.Settings.Default.TextureHeight.ToString();
            chkTangentSpace.IsChecked = Properties.Settings.Default.UseTangentSpace;
            chkCosineDistribution.IsChecked = Properties.Settings.Default.UseCosineDistribution;
            chkGenerateBentNormal.IsChecked = Properties.Settings.Default.GenerateBentNormalMap;
            chkGenerateOcclusion.IsChecked = Properties.Settings.Default.GenerateOcclusionMap;
            chkInvertX.IsChecked = Properties.Settings.Default.SwizzleX < 0;
            chkInvertY.IsChecked = Properties.Settings.Default.SwizzleY < 0;
            chkInvertZ.IsChecked = Properties.Settings.Default.SwizzleZ < 0;
            //chkEnhancedTangentProcessing.IsChecked = Properties.Settings.Default.EnhancedTangentProcessing;
            chkClampOcclusion.IsChecked = !Properties.Settings.Default.ClampOcclusion;

            // Load fast preview option.
            fastPreviewEnabled = Properties.Settings.Default.FastPreview;
            chkFastPreview.IsChecked = fastPreviewEnabled;
            previewInterval = fastPreviewEnabled ? 50 : 500;

            // Load additional baking parameters.
            txtRaySampleCount.Text = Properties.Settings.Default.RaySampleCount.ToString();
            txtMaxRayDistance.Text = Properties.Settings.Default.MaxRayDistance.ToString();
            txtOcclusionThreshold.Text = Properties.Settings.Default.OcclusionThreshold.ToString();
            txtRayOriginBias.Text = Properties.Settings.Default.RayOriginBias.ToString();

            // Load last-used folders.
            lastModelFolder = Properties.Settings.Default.LastModelFolder;
            lastAlphaFolder = Properties.Settings.Default.LastAlphaFolder;

            // Load last selected model file and import scene.
            string lastModelPath = Properties.Settings.Default.LastModelPath;
            if (!string.IsNullOrEmpty(lastModelPath) && System.IO.File.Exists(lastModelPath))
            {
                modelPath = lastModelPath;
                txtModelPath.Text = modelPath;
                btnLoadModel.Content = "Clear Model"; // Update button text
                AssimpContext context = new AssimpContext();
                loadedScene = context.ImportFile(modelPath,
                    PostProcessSteps.Triangulate |
                    PostProcessSteps.GenerateUVCoords |
                    PostProcessSteps.CalculateTangentSpace);

                // Populate the mesh dropdown.
                cmbMeshSelection.Items.Clear();
                for (int i = 0; i < loadedScene.MeshCount; i++)
                {
                    var mesh = loadedScene.Meshes[i];
                    string meshName = !string.IsNullOrEmpty(mesh.Name) ? mesh.Name : $"Mesh {i}";
                    cmbMeshSelection.Items.Add(meshName);
                }
                cmbMeshSelection.SelectedIndex = 0;
                selectedMeshIndex = 0;
                PopulateUVOptionsForMesh(loadedScene.Meshes[selectedMeshIndex]);
            }
            else
            {
                // No model loaded.
                modelPath = "";
                txtModelPath.Text = "No model loaded";
                btnLoadModel.Content = "Load Model";
            }

            // Load last selected alpha texture using the AlphaTextureHelper.
            string lastAlphaPath = Properties.Settings.Default.LastAlphaPath;
            if (!string.IsNullOrEmpty(lastAlphaPath) && System.IO.File.Exists(lastAlphaPath))
            {
                txtAlphaPath.Text = lastAlphaPath;
                AlphaTextureHelper.LoadAlphaTexture(lastAlphaPath);
                btnLoadAlphaTexture.Content = "Clear Alpha Texture"; // Update button text
            }
            else
            {
                txtAlphaPath.Text = "No alpha texture loaded";
                btnLoadAlphaTexture.Content = "Load Alpha Texture";
            }
        }

        /// <summary>
        /// Saves persistent settings and cancels ongoing operations on window closing.
        /// </summary>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            // Cancel any running bake operation.
            bakeCancellationTokenSource?.Cancel();

            // Save texture dimensions.
            if (int.TryParse(txtWidth.Text, out int w))
                Properties.Settings.Default.TextureWidth = w;
            if (int.TryParse(txtHeight.Text, out int h))
                Properties.Settings.Default.TextureHeight = h;

            // Save baking and UI settings.
            Properties.Settings.Default.UseTangentSpace = (chkTangentSpace.IsChecked == true);
            Properties.Settings.Default.UseCosineDistribution = (chkCosineDistribution.IsChecked == true);
            Properties.Settings.Default.GenerateBentNormalMap = (chkGenerateBentNormal.IsChecked == true);
            Properties.Settings.Default.GenerateOcclusionMap = (chkGenerateOcclusion.IsChecked == true);
            Properties.Settings.Default.SwizzleX = (chkInvertX.IsChecked == true) ? -1f : 1f;
            Properties.Settings.Default.SwizzleY = (chkInvertY.IsChecked == true) ? -1f : 1f;
            Properties.Settings.Default.SwizzleZ = (chkInvertZ.IsChecked == true) ? -1f : 1f;
            //Properties.Settings.Default.EnhancedTangentProcessing = (chkEnhancedTangentProcessing.IsChecked == true);
            Properties.Settings.Default.ClampOcclusion = (chkClampOcclusion.IsChecked == false);

            // Save dark mode state and highlight color.
            Properties.Settings.Default.IsDarkMode = menuDarkModeToggle.IsChecked == true;
            if (Resources["ButtonBackgroundBrush"] is SolidColorBrush brush)
                Properties.Settings.Default.HighlightColor = brush.Color.ToString();

            // Save additional baking parameters.
            if (int.TryParse(txtRaySampleCount.Text, out int sampleCount))
                Properties.Settings.Default.RaySampleCount = sampleCount;
            if (float.TryParse(txtMaxRayDistance.Text, out float maxDist))
                Properties.Settings.Default.MaxRayDistance = maxDist;
            if (float.TryParse(txtOcclusionThreshold.Text, out float occThresh))
                Properties.Settings.Default.OcclusionThreshold = occThresh;
            if (float.TryParse(txtRayOriginBias.Text, out float originBias))
                Properties.Settings.Default.RayOriginBias = originBias;

            Properties.Settings.Default.FastPreview = fastPreviewEnabled;
            Properties.Settings.Default.LastModelFolder = lastModelFolder;
            Properties.Settings.Default.LastAlphaFolder = lastAlphaFolder;

            // Save the loaded model and alpha texture paths based on current state.
            // If the user cleared the model, then modelPath is empty.
            Properties.Settings.Default.LastModelPath = modelPath;
            // Similarly, if the alpha texture is cleared, txtAlphaPath.Text should say "No alpha texture loaded".
            Properties.Settings.Default.LastAlphaPath = txtAlphaPath.Text == "No alpha texture loaded" ? "" : txtAlphaPath.Text;

            Properties.Settings.Default.Save();
        }

        #endregion

        #region Assimp Node Transform Helpers
        /// <summary>
        /// Recursively searches for the node that directly references the specified mesh index.
        /// </summary>
        private Assimp.Node FindMeshNode(Assimp.Node node, int meshIndex)
        {
            if (node.MeshIndices.Contains(meshIndex))
                return node;
            foreach (var child in node.Children)
            {
                var result = FindMeshNode(child, meshIndex);
                if (result != null)
                    return result;
            }
            return null;
        }
        #endregion

        #region UI Event Handlers
        /// <summary>
        /// Toggles the fast preview mode and adjusts the preview interval.
        /// </summary>
        private void fastPreviewModeToggle_Click(object sender, RoutedEventArgs e)
        {
            fastPreviewEnabled = chkFastPreview.IsChecked == true;
            previewInterval = fastPreviewEnabled ? 50 : 500;
        }

        private void chkBackfaceProcessing_Checked(object sender, RoutedEventArgs e)
        {
            AARasterizer.AllowBackFacing = true;
        }

        private void chkBackfaceProcessing_Unchecked(object sender, RoutedEventArgs e)
        {
            AARasterizer.AllowBackFacing = false;
        }


        /// <summary>
        /// Toggles dark mode and updates various UI resource brushes accordingly.
        /// </summary>
        private void menuDarkModeToggle_Click(object sender, RoutedEventArgs e)
        {
            if (menuDarkModeToggle.IsChecked == true)
            {
                Resources["WindowBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#1E1E1E"));
                Resources["MenuBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#2D2D30"));
                Resources["ControlBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#2D2D30"));
                Resources["ControlForegroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#E1E1E1"));
                if (!(Resources["ButtonBackgroundBrush"] is SolidColorBrush))
                {
                    Resources["ButtonBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#0078D7"));
                    Resources["GroupBoxHeaderForegroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#0078D7"));
                    Resources["ButtonHoverBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#005A9E"));
                    Resources["ButtonPressedBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#004578"));
                }
                Resources["TextBoxBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#3C3C3C"));
                Resources["TextBoxForegroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#E1E1E1"));
                Resources["TextBoxBorderBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#555555"));
                Resources["GroupBoxBorderBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#3C3C3C"));
            }
            else
            {
                Resources["WindowBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#F3F3F3"));
                Resources["MenuBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#F3F3F3"));
                Resources["ControlBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("White"));
                Resources["ControlForegroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("Black"));
                if (!(Resources["ButtonBackgroundBrush"] is SolidColorBrush))
                {
                    Resources["ButtonBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#0078D7"));
                    Resources["GroupBoxHeaderForegroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#0078D7"));
                    Resources["ButtonHoverBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#005A9E"));
                    Resources["ButtonPressedBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#004578"));
                }
                Resources["TextBoxBackgroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("White"));
                Resources["TextBoxForegroundBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("Black"));
                Resources["TextBoxBorderBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#CCCCCC"));
                Resources["GroupBoxBorderBrush"] = new SolidColorBrush((MediaColor)MediaColorConverter.ConvertFromString("#DDDDDD"));
            }
        }

        /// <summary>
        /// Opens a color dialog for selecting a highlight color and updates the UI resources.
        /// </summary>
        private void menuSetHighlightColor_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new WinForms.ColorDialog();
            if (Resources["ButtonBackgroundBrush"] is SolidColorBrush brush)
            {
                System.Drawing.Color initialColor = System.Drawing.Color.FromArgb(brush.Color.A, brush.Color.R, brush.Color.G, brush.Color.B);
                dlg.Color = initialColor;
            }
            if (dlg.ShowDialog() == WinForms.DialogResult.OK)
            {
                System.Drawing.Color selected = dlg.Color;
                MediaColor newHighlight = MediaColor.FromArgb(selected.A, selected.R, selected.G, selected.B);
                Resources["ButtonBackgroundBrush"] = new SolidColorBrush(newHighlight);
                Resources["GroupBoxHeaderForegroundBrush"] = new SolidColorBrush(newHighlight);
                Resources["ButtonHoverBackgroundBrush"] = new SolidColorBrush(DarkenColor(newHighlight, 0.8));
                Resources["ButtonPressedBackgroundBrush"] = new SolidColorBrush(DarkenColor(newHighlight, 0.6));
            }
        }

        /// <summary>
        /// Opens a file dialog to load a 3D model, imports the scene, and populates mesh and UV options.
        /// </summary>
        private void btnLoadModel_Click(object sender, RoutedEventArgs e)
        {
            // If a model is already loaded, clear it.
            if (!string.IsNullOrEmpty(modelPath))
            {
                modelPath = "";
                loadedScene = null;
                txtModelPath.Text = "No model loaded";
                cmbMeshSelection.Items.Clear();
                btnLoadModel.Content = "Load Model";
                return;
            }

            // No model loaded: open file dialog to load one.
            var dlg = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "3D Models|*.fbx|All Files|*.*",
                InitialDirectory = string.IsNullOrEmpty(lastModelFolder)
                    ? Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments)
                    : lastModelFolder
            };
            if (dlg.ShowDialog() == true)
            {
                modelPath = dlg.FileName;
                txtModelPath.Text = modelPath;
                btnLoadModel.Content = "Clear Model";
                lastModelFolder = System.IO.Path.GetDirectoryName(modelPath);
                Properties.Settings.Default.LastModelPath = modelPath;

                // Import the model.
                AssimpContext context = new AssimpContext();
                loadedScene = context.ImportFile(modelPath,
                    PostProcessSteps.Triangulate |
                    PostProcessSteps.GenerateUVCoords |
                    PostProcessSteps.CalculateTangentSpace);

                // Populate the mesh dropdown.
                cmbMeshSelection.Items.Clear();
                for (int i = 0; i < loadedScene.MeshCount; i++)
                {
                    var mesh = loadedScene.Meshes[i];
                    string meshName = !string.IsNullOrEmpty(mesh.Name) ? mesh.Name : $"Mesh {i}";
                    cmbMeshSelection.Items.Add(meshName);
                }
                cmbMeshSelection.SelectedIndex = 0;
                selectedMeshIndex = 0;
                PopulateUVOptionsForMesh(loadedScene.Meshes[selectedMeshIndex]);
            }
        }


        /// <summary>
        /// Updates the UV selection dropdowns when a new mesh is selected.
        /// </summary>
        private void cmbMeshSelection_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (cmbMeshSelection.SelectedIndex >= 0 && loadedScene != null)
            {
                selectedMeshIndex = cmbMeshSelection.SelectedIndex;
                PopulateUVOptionsForMesh(loadedScene.Meshes[selectedMeshIndex]);
            }
        }

        /// <summary>
        /// Opens a file dialog to load an external alpha texture.
        /// Delegates loading and caching to the AlphaTextureHelper.
        /// </summary>
        private void btnLoadAlphaTexture_Click(object sender, RoutedEventArgs e)
        {
            // If an alpha texture is already loaded, clear it.
            if (!string.IsNullOrEmpty(txtAlphaPath.Text) && txtAlphaPath.Text != "No alpha texture loaded")
            {
                txtAlphaPath.Text = "No alpha texture loaded";
                AlphaTextureHelper.ClearAlphaTexture();
                btnLoadAlphaTexture.Content = "Load Alpha Texture";
                return;
            }

            // No alpha texture loaded: open file dialog to load one.
            var dlg = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "Image Files|*.png;*.jpg;*.bmp",
                InitialDirectory = string.IsNullOrEmpty(lastAlphaFolder)
                    ? Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments)
                    : lastAlphaFolder
            };
            if (dlg.ShowDialog() == true)
            {
                AlphaTextureHelper.LoadAlphaTexture(dlg.FileName);
                txtAlphaPath.Text = dlg.FileName;
                btnLoadAlphaTexture.Content = "Clear Alpha Texture";
                lastAlphaFolder = System.IO.Path.GetDirectoryName(dlg.FileName);
                Properties.Settings.Default.LastAlphaPath = dlg.FileName;
            }
        }


        /// <summary>
        /// Initiates the baking process, updating the UI during processing and saving the resulting maps.
        /// </summary>
        private async void btnBake_Click(object sender, RoutedEventArgs e)
        {
            // Cancel any ongoing bake if one is active.
            if (bakeCancellationTokenSource != null && !bakeCancellationTokenSource.IsCancellationRequested)
            {
                bakeCancellationTokenSource.Cancel();
                btnBake.Content = "Bake Maps";
                return;
            }

            btnBake.Content = "Cancel Bake";
            bakeCancellationTokenSource = new CancellationTokenSource();

            if (int.TryParse(txtRaySampleCount.Text, out int sampleCount))
                raySampleCount = sampleCount;
            if (float.TryParse(txtMaxRayDistance.Text, out float maxDist))
                maxRayDistance = maxDist;
            if (float.TryParse(txtOcclusionThreshold.Text, out float occThresh))
                occlusionThreshold = occThresh;
            if (float.TryParse(txtRayOriginBias.Text, out float originBias))
                rayOriginBias = originBias;

            if (!int.TryParse(txtWidth.Text, out textureWidth) ||
                !int.TryParse(txtHeight.Text, out textureHeight))
            {
                System.Windows.MessageBox.Show("Invalid texture dimensions.");
                return;
            }

            useTangentSpace = (chkTangentSpace.IsChecked == true);
            useCosineDistribution = (chkCosineDistribution.IsChecked == true);
            generateBentNormalMap = (chkGenerateBentNormal.IsChecked == true);
            generateOcclusionMap = (chkGenerateOcclusion.IsChecked == true);
            //useEnhancedTangentProcessing = (chkEnhancedTangentProcessing.IsChecked == true);

            float swizX = (chkInvertX.IsChecked == true) ? -1f : 1f;
            float swizY = (chkInvertY.IsChecked == true) ? -1f : 1f;
            float swizZ = (chkInvertZ.IsChecked == true) ? -1f : 1f;
            swizzle = new Vector3(swizX, swizY, swizZ);

            try
            {
                var progress = new Progress<double>(p =>
                {
                    this.Title = $"Baking... {p:P0}";
                    taskbarInfo.ProgressState = System.Windows.Shell.TaskbarItemProgressState.Normal;
                    taskbarInfo.ProgressValue = p;
                });
                int bentUVIndex = cmbBentMapUV.SelectedIndex;
                int alphaUVIndex = cmbAlphaMapUV.SelectedIndex;
                bool clampOcclusion = chkClampOcclusion.IsChecked == false;

                // Start the baking process using the BakingEngine.
                BakeResult bakeResult = await BakingEngine.BakeBentNormalMapAsync(
                    modelPath, textureWidth, textureHeight,
                    useTangentSpace, useCosineDistribution,
                    generateBentNormalMap, generateOcclusionMap,
                    swizzle, progress, bakeCancellationTokenSource.Token, clampOcclusion,
                    bentUVIndex, alphaUVIndex, selectedMeshIndex, loadedScene,
                    raySampleCount,
                    maxRayDistance,
                    occlusionThreshold,
                    rayOriginBias,
                    useEnhancedTangentProcessing,
                    fastPreviewEnabled ? 50 : 500,
                    AlphaTextureHelper.SampleAlpha,
                    bmp => imgPreview.Source = BitmapToImageSource(bmp),
                    title => this.Title = title,
                    action => Dispatcher.Invoke(action)
                );

                btnBake.Content = "Bake Maps";
                bakeCancellationTokenSource = null;
                taskbarInfo.ProgressState = System.Windows.Shell.TaskbarItemProgressState.None;
                this.Title = "Bake Complete!";

                // Save the generated maps.
                if (generateBentNormalMap && generateOcclusionMap)
                {
                    var dlgBent = new Microsoft.Win32.SaveFileDialog
                    {
                        Filter = "PNG Image|*.png|JPEG Image|*.jpg|Bitmap Image|*.bmp",
                        FileName = "BentNormalMap.png"
                    };
                    if (dlgBent.ShowDialog() == true)
                    {
                        System.Drawing.Imaging.ImageFormat format = System.Drawing.Imaging.ImageFormat.Png;
                        string ext = System.IO.Path.GetExtension(dlgBent.FileName).ToLower();
                        if (ext == ".jpg" || ext == ".jpeg")
                            format = System.Drawing.Imaging.ImageFormat.Jpeg;
                        else if (ext == ".bmp")
                            format = System.Drawing.Imaging.ImageFormat.Bmp;
                        bakeResult.BentMap.Save(dlgBent.FileName, format);
                        System.Windows.MessageBox.Show("Bent Normal Map saved to: " + dlgBent.FileName);
                    }

                    var dlgOcc = new Microsoft.Win32.SaveFileDialog
                    {
                        Filter = "PNG Image|*.png|JPEG Image|*.jpg|Bitmap Image|*.bmp",
                        FileName = "OcclusionMap.png"
                    };
                    if (dlgOcc.ShowDialog() == true)
                    {
                        System.Drawing.Imaging.ImageFormat format = System.Drawing.Imaging.ImageFormat.Png;
                        string ext = System.IO.Path.GetExtension(dlgOcc.FileName).ToLower();
                        if (ext == ".jpg" || ext == ".jpeg")
                            format = System.Drawing.Imaging.ImageFormat.Jpeg;
                        else if (ext == ".bmp")
                            format = System.Drawing.Imaging.ImageFormat.Bmp;
                        bakeResult.OccMap.Save(dlgOcc.FileName, format);
                        System.Windows.MessageBox.Show("Occlusion Map saved to: " + dlgOcc.FileName);
                    }
                }
                else
                {
                    var dlg = new Microsoft.Win32.SaveFileDialog
                    {
                        Filter = "PNG Image|*.png|JPEG Image|*.jpg|Bitmap Image|*.bmp",
                        FileName = generateOcclusionMap ? "OcclusionMap.png" : "BentNormalMap.png"
                    };
                    if (dlg.ShowDialog() == true)
                    {
                        System.Drawing.Imaging.ImageFormat format = System.Drawing.Imaging.ImageFormat.Png;
                        string ext = System.IO.Path.GetExtension(dlg.FileName).ToLower();
                        if (ext == ".jpg" || ext == ".jpeg")
                            format = System.Drawing.Imaging.ImageFormat.Jpeg;
                        else if (ext == ".bmp")
                            format = System.Drawing.Imaging.ImageFormat.Bmp;
                        bakeResult.PreviewBmp.Save(dlg.FileName, format);
                        System.Windows.MessageBox.Show("Image saved to: " + dlg.FileName);
                    }
                }
            }
            catch (OperationCanceledException)
            {
                btnBake.Content = "Bake Maps";
                bakeCancellationTokenSource = null;
                taskbarInfo.ProgressState = System.Windows.Shell.TaskbarItemProgressState.None;
                this.Title = "Normal Smith";
            }
            catch (Exception ex)
            {
                System.Windows.MessageBox.Show("Error: " + ex.Message);
                btnBake.Content = "Bake Maps";
                bakeCancellationTokenSource = null;
            }
        }
        private void PopulateUVOptionsForMesh(Assimp.Mesh mesh)
        {
            int uvCount = mesh.TextureCoordinateChannelCount;
            cmbBentMapUV.Items.Clear();
            cmbAlphaMapUV.Items.Clear();
            for (int i = 0; i < uvCount; i++)
            {
                cmbBentMapUV.Items.Add("UV " + i);
                cmbAlphaMapUV.Items.Add("UV " + i);
            }
            // Set defaults: use UV1 for map baking (if available) and UV0 for alpha map.
            cmbBentMapUV.SelectedIndex = uvCount > 1 ? 1 : 0;
            cmbAlphaMapUV.SelectedIndex = 0;
        }
        #endregion

        #region Utility Methods
        /// <summary>
        /// Converts a System.Drawing.Bitmap to a WPF BitmapSource.
        /// </summary>
        private BitmapSource BitmapToImageSource(System.Drawing.Bitmap bitmap)
        {
            IntPtr hBitmap = bitmap.GetHbitmap();
            try
            {
                BitmapSource wpfBitmap = Imaging.CreateBitmapSourceFromHBitmap(
                    hBitmap,
                    IntPtr.Zero,
                    Int32Rect.Empty,
                    BitmapSizeOptions.FromEmptyOptions());
                return wpfBitmap;
            }
            finally
            {
                DeleteObject(hBitmap);
            }
        }
        private MediaColor DarkenColor(MediaColor color, double factor)
        {
            return MediaColor.FromArgb(
                color.A,
                (byte)(color.R * factor),
                (byte)(color.G * factor),
                (byte)(color.B * factor)
            );
        }

        // Import the GDI function to release HBITMAP handles.
        [DllImport("gdi32.dll")]
        private static extern bool DeleteObject(IntPtr hObject);
        #endregion
    }
}
