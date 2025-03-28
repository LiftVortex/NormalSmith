# Normal Smith

## Overview

Normal Smith is a Windows Presentation Foundation (WPF) application written in C# designed to bake lighting-related texture maps—specifically bent normal maps and ambient occlusion maps—from 3D models. Using the Assimp library for model importing, the application processes 3D mesh data, computes per-pixel lighting information through ray-casting techniques, and outputs texture maps that can be used in game development and real-time rendering workflows.

## Screenshot

![Main UI](https://github.com/user-attachments/assets/344b2c02-fe39-4b44-a75d-da1121e015b0)  
*Example of the Normal Smith main interface with a loaded model and live preview when baking a bent normal texture.*

## Features

- **3D Model Importing:**  
  Supports FBX via the Assimp library.

- **Mesh and UV Channel Selection:**  
  Easily select from multiple meshes within a model and choose the appropriate UV channels for baking maps and alpha textures.

- **Bent Normal Map & Ambient Occlusion Baking:**  
  Uses ray-casting from each pixel on the UV space to compute ambient occlusion and bent normal directions.

- **Texture Output Settings:**  
  The size of the output texture maps can be customised and a super sampling factor can be applied to help remove noise at lower resolutions.

- **Map Types:**  
  Can select to output either a Bent Normal Map, Occlusion Map or both.

- **Customisable Sampling Options:**  
  Configure ray sample count, maximum ray distance, occlusion threshold, and ray origin bias. Options for cosine-weighted sampling, tangent space and back-facing triangle processing are available.

- **Efficient BVH Acceleration Structure:**  
  Builds a bounding volume hierarchy (BVH) for rapid occlusion testing during ray intersection.

- **Alpha Texture Integration:**  
  Supports loading an external alpha texture to account for transparency in occlusion tests.

- **Real-Time Preview & Progress Updates:**  
  Displays a live preview of the baking process to show progress and displays a percentage complete indicator.

- **User Interface Customisation:**  
  Toggle between light and dark modes, adjust highlight colours, and enable fast preview for more responsive updates.

- **Persistent Settings:**  
  Remembers user preferences, including texture dimensions, recent folders, and map generation options between sessions.

## How to Use

1. **Launch the Application:**  
   Download the compiled Windows executable available in [releases](https://github.com/Normal-Smith/NormalSmith/releases/latest).

2. **Load a 3D Model:**  
   Click the **Load Model** button to select a 3D model file (`.fbx`). The application will import the model and populate the mesh selection dropdown.

3. **Select Mesh and UV Channels:**  
   Use the dropdowns to select the desired mesh and choose the UV channels for the bent normal/occlusion map and alpha texture.

4. **Load an Alpha Texture (Optional):**  
   If your model uses transparency, load an external alpha texture using the **Load Alpha Texture** button.

5. **Configure Baking Options:**  
   Adjust settings such as texture dimensions, ray sample count, maximum ray distance, occlusion threshold, and whether to use tangent space or enhanced tangent processing.

6. **Bake the Maps:**  
   Click the **Bake Maps** button to start the baking process. You can monitor progress via the live preview and window title updates.

7. **Save the Output:**  
   After the bake completes, you can now click the Save Maps button which will ask where to save them to.

## Configuration / Settings

The application offers various configuration options, please read the [Wiki](https://github.com/Normal-Smith/NormalSmith/wiki/Normal-Smith-Options) page for more details:

- **Texture Dimensions:**  
  Set the width and height of the output texture and super sampling factor.

- **Sampling Options:**  
  Configure ray sample count, maximum ray distance, occlusion threshold, and ray origin bias.

- **Map Generation Options:**  
  Enable or disable tangent space transformation, cosine distribution, and back-facing triangle processing.

- **Swizzle Options:**  
  Invert X, Y, or Z channels in the final colour output.

- **Preview Options:**  
  Toggle fast preview mode for more responsive live updates.

- **UI Customisation:**  
  Toggle dark mode and set highlight colours through the menu options.

User settings are saved automatically between sessions.

## Troubleshooting

If you are having issues, please check the [Troubleshooting](https://github.com/Normal-Smith/NormalSmith/wiki/Troubleshooting) page first before submitting an issue on GitHub.

## Building from Source
### Prerequisites

- **.NET Framework / .NET Core:**  
  The application is built using C# and WPF, so ensure you have a compatible version of the .NET SDK installed.

- **Visual Studio:**  
  Recommended for building and debugging the application.

- **AssimpNet Library:**  
  The project uses Assimp for model importing. Make sure the AssimpNet NuGet package is referenced.

### Installation

1. **Clone the Repository:**  
    ```
    git clone https://github.com/Normal-Smith/NormalSmith.git
    cd NormalSmith
    ```

2. **Open the Solution:**  
  Open NormalSmith.sln in Visual Studio.

3. **Restore NuGet Packages:**  
  Visual Studio should automatically restore NuGet packages (including AssimpNet).

### Building and Running

1. **Build the Project:**  
   In Visual Studio, build the solution (`Build > Build Solution`).

2. **Run the Application:**  
   Start the application via `Debug > Start Debugging` or run the compiled executable from the output directory.

## Contributing

Contributions are welcome! If you have suggestions or bug fixes, please:

- Fork the repository.
- Create a feature branch.
- Commit your changes and open a pull request.

For major changes, please open an issue first to discuss what you would like to change.

## Known Issues / Roadmap

Check the [Issues](https://github.com/Normal-Smith/NormalSmith/issues) page for the latest known issues.

## License

This project is licensed under the [MIT License](https://github.com/Normal-Smith/NormalSmith?tab=MIT-1-ov-file#readme).

## Acknowledgements / Credits

- **Assimp & AssimpNet:**  
  Thanks to the Assimp community for providing the robust model importing capabilities.

- **WPF & .NET Community:**  
  Appreciation to the developers and contributors who continue to improve the .NET ecosystem.

- **Inspiration and Contributions:**  
  Thanks to all contributors and users who provide feedback and help improve Normal Smith.
