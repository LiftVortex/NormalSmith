param(
    [string]$InstallDir = $PSScriptRoot
)

# Enable detailed verbose messages
$VerbosePreference = 'Continue'

Write-Host "============================"
Write-Host " NormalSmith Updater Script"
Write-Host "============================"
Write-Host "Script is running from: $PSScriptRoot"
Write-Host "InstallDir (destination): $InstallDir"
Write-Host " "

# --- Step 1: Read the configuration file ---
Write-Host "[Step 1] Looking for updater.config.json ..."
$configPath = Join-Path $PSScriptRoot "updater.config.json"
Write-Host "   > Checking path: $configPath"

if (Test-Path $configPath) {
    Write-Host "   > Found config file. Reading and parsing JSON content..."
    $config = Get-Content $configPath | ConvertFrom-Json
    $downloadUrl = $config.downloadUrl
    Write-Host "   > Download URL extracted from config file: $downloadUrl"
}
else {
    Write-Host "   > ERROR: Config file updater.config.json not found at $configPath."
    exit 1
}

Write-Host " "

# --- Step 2: Define temporary paths ---
Write-Host "[Step 2] Defining temporary file/folder paths..."
$tempZip = Join-Path $env:TEMP "NormalSmith-Build.zip"
$tempExtractDir = Join-Path $env:TEMP "NormalSmith_Update"

Write-Host "   > Initial $tempExtractDir (may have short-path or trailing slash issues): $tempExtractDir"

# Remove any trailing slash from $tempExtractDir
$tempExtractDir = $tempExtractDir.TrimEnd('\')

# If the directory exists, remove it now (we want a fresh directory).
if (Test-Path $tempExtractDir) {
    Write-Host "   > Removing existing folder (to ensure a clean extract): $tempExtractDir"
    Remove-Item $tempExtractDir -Recurse -Force
}

# Create the directory so we can do Get-Item on it
Write-Host "   > Creating directory: $tempExtractDir"
New-Item -ItemType Directory -Path $tempExtractDir | Out-Null

# Force $tempExtractDir to the long-path form
$tempExtractDir = (Get-Item $tempExtractDir).FullName

Write-Host "   > Normalized long path for $tempExtractDir"
Write-Host "   > ZIP file will be downloaded to: $tempZip"
Write-Host " "

# --- Step 3: Download the update zip ---
Write-Host "[Step 3] Downloading update from: $downloadUrl ..."
try {
    Invoke-WebRequest -Uri $downloadUrl -OutFile $tempZip -ErrorAction Stop
    Write-Host "   > Download successful!"
}
catch {
    Write-Host "   > ERROR: Failed to download from $downloadUrl"
    Write-Host "   > Exception: $_"
    exit 1
}

Write-Host "   > Pausing briefly to ensure the main application has fully exited..."
Start-Sleep -Seconds 2
Write-Host " "

# --- Step 4: Extract the zip to the fresh directory ---
Write-Host "[Step 4] Extracting ZIP to temporary directory: $tempExtractDir"
try {
    Expand-Archive -Path $tempZip -DestinationPath $tempExtractDir -Force
    Write-Host "   > Extraction complete!"
}
catch {
    Write-Host "   > ERROR: Could not expand ZIP file."
    Write-Host "   > Exception: $_"
    exit 1
}
Write-Host " "

# --- Step 5: Copy the updated files to the installation folder ---
Write-Host "[Step 5] Copying updated files to: $InstallDir"

Get-ChildItem -Path $tempExtractDir -Recurse | ForEach-Object {
    $sourcePath = $_.FullName

    # Show debug info about the substring replacement
    Write-Host "DEBUG: Attempting to replace '$tempExtractDir' with '$InstallDir' in '$sourcePath'"
    $dest = $sourcePath.Replace($tempExtractDir, $InstallDir)
    Write-Host "DEBUG: After Replace => $dest"

    if ([string]::Equals($sourcePath, $dest, [System.StringComparison]::InvariantCultureIgnoreCase)) {
        Write-Host "   > Skipping (source and destination are the same)."
        return
    }

    if ($_.PSIsContainer) {
        if (-not (Test-Path $dest)) {
            Write-Host "   > Creating directory: $dest"
            New-Item -ItemType Directory -Path $dest | Out-Null
        }
        else {
            Write-Host "   > Directory already exists: $dest"
        }
    }
    else {
        Write-Host "   > Copying file to: $dest"
        Copy-Item -Path $sourcePath -Destination $dest -Force
    }
}

Write-Host " "

# --- Step 6: Clean up temporary files ---
Write-Host "[Step 6] Removing temporary files..."

Write-Host "   > Removing ZIP file: $tempZip"
Remove-Item $tempZip -Force -ErrorAction SilentlyContinue

Write-Host "   > Removing temp extract directory: $tempExtractDir"
Remove-Item $tempExtractDir -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "   > Temporary files cleaned up."
Write-Host " "

# --- Step 7: Relaunch the updated application ---
Write-Host "[Step 7] Launching the updated NormalSmith application..."
$exePath = Join-Path $InstallDir "NormalSmith.exe"
Write-Host "   > Executable Path: $exePath"

if (Test-Path $exePath) {
    Write-Host "   > Starting application..."
    Start-Process $exePath
}
else {
    Write-Host "   > ERROR: Could not find NormalSmith.exe at: $exePath"
}

Write-Host " "
Write-Host "============================"
Write-Host " Update Process Completed  "
Write-Host "============================"
Write-Host " "
