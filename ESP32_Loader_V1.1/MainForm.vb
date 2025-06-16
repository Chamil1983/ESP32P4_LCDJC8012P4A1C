Imports System.IO
Imports System.IO.Ports
Imports System.Diagnostics
Imports System.ComponentModel
Imports System.Text
Imports System.Threading

Public Class MainForm
    ' UI control to show selected partition: txtPartitionPath
    ' UI button for custom partition: btnSelectPartition
    ' UI output textbox: txtOutput

    Private customPartitionFilePath As String = ""
    Private partitionBackupPath As String = ""
    Private arduinoCliPath As String = ""
    Private sketchPath As String = ""
    Private arduinoUserDir As String
    Private boardList As New List(Of ArduinoBoard)
    Private boardConfig As ArduinoBoardConfig
    Private boardsTxtPath As String = Application.StartupPath & "\boards.txt"
    Private arduinoCliVersion As String = ""

    ' Background workers for compile and upload
    Private WithEvents bgwCompile As New BackgroundWorker With {.WorkerReportsProgress = True, .WorkerSupportsCancellation = True}
    Private WithEvents bgwUpload As New BackgroundWorker With {.WorkerReportsProgress = True, .WorkerSupportsCancellation = True}

    ' Stages for progress tracking
    Private cliStagesCompile As String() = {
        "Detecting libraries",
        "Compiling sketch",
        "Compiling core",
        "Compiling libraries",
        "Linking everything together",
        "Building into",
        "Sketch uses"
    }
    Private cliStagesUpload As String() = {
        "Uploading",
        "Writing at",
        "Hash of data verified",
        "Hard resetting via RTS",
        "Leaving"
    }

    Public Sub New()
        InitializeComponent()
        arduinoUserDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "Arduino15")
        If Not Directory.Exists(arduinoUserDir) Then
            arduinoUserDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Arduino15")
            If Not Directory.Exists(arduinoUserDir) Then
                arduinoUserDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "Local", "Arduino15")
            End If
        End If
    End Sub

    Private Sub MainForm_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        RefreshComPorts()
        LoadBoardsTxt()
        txtPartitionPath.Text = "No custom partition file selected"
    End Sub

    Private Sub btnRefreshPorts_Click(sender As Object, e As EventArgs) Handles btnRefreshPorts.Click
        RefreshComPorts()
    End Sub

    Private Sub btnSelectCli_Click(sender As Object, e As EventArgs) Handles btnSelectCli.Click
        Using ofd As New OpenFileDialog()
            ofd.Filter = "arduino-cli.exe|arduino-cli.exe"
            If ofd.ShowDialog() = DialogResult.OK Then
                arduinoCliPath = ofd.FileName
                txtCliPath.Text = arduinoCliPath
                GetArduinoCliVersion()
                DetectEsp32Core()
                DetectBoardList()
            End If
        End Using
    End Sub

    Private Sub btnSelectPartition_Click(sender As Object, e As EventArgs) Handles btnSelectPartition.Click
        Using ofd As New OpenFileDialog()
            ofd.Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*"
            ofd.Title = "Select custom partitions.csv"
            If ofd.ShowDialog() = DialogResult.OK Then
                customPartitionFilePath = ofd.FileName
                txtPartitionPath.Text = customPartitionFilePath

                ' Verify it's a valid partition file
                Try
                    Dim content As String = File.ReadAllText(customPartitionFilePath)
                    If content.Contains("nvs") AndAlso content.Contains("app") Then
                        AppendOutputLine("✅ Valid partition file detected.")
                        AppendOutputLine("Custom partition will be used when 'PartitionScheme=custom' is selected.")
                    Else
                        AppendOutputLine("⚠️ Warning: This might not be a valid partition file.")
                        AppendOutputLine("It should contain sections for 'nvs' and 'app' partitions.")
                    End If
                Catch ex As Exception
                    AppendOutputLine("Error reading partition file: " & ex.Message)
                End Try
            End If
        End Using
    End Sub

    Private Sub GetArduinoCliVersion()
        If String.IsNullOrEmpty(arduinoCliPath) Then Return

        Try
            Dim psi As New ProcessStartInfo(arduinoCliPath, "version")
            psi.RedirectStandardOutput = True
            psi.UseShellExecute = False
            psi.CreateNoWindow = True
            Dim p As Process = Process.Start(psi)
            arduinoCliVersion = p.StandardOutput.ReadToEnd().Trim()
            p.WaitForExit()

            ' Log the version
            txtOutput.AppendText("Arduino CLI Version: " & arduinoCliVersion & vbCrLf)
        Catch ex As Exception
            MessageBox.Show("Error checking Arduino CLI version: " & ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error)
        End Try
    End Sub

    Private Sub btnSelectSketch_Click(sender As Object, e As EventArgs) Handles btnSelectSketch.Click
        Using fbd As New FolderBrowserDialog()
            If fbd.ShowDialog() = DialogResult.OK Then
                sketchPath = fbd.SelectedPath
                txtSketchPath.Text = sketchPath

                ' Check if sketch directory contains a custom partition file
                Dim sketchPartitionFile = Path.Combine(sketchPath, "partitions.csv")
                If File.Exists(sketchPartitionFile) Then
                    AppendOutputLine("Found partitions.csv in sketch directory.")
                    If MessageBox.Show("Found partitions.csv in sketch directory. Use it as custom partition?",
                                     "Custom Partition Found", MessageBoxButtons.YesNo,
                                     MessageBoxIcon.Question) = DialogResult.Yes Then
                        customPartitionFilePath = sketchPartitionFile
                        txtPartitionPath.Text = customPartitionFilePath
                    End If
                End If
            End If
        End Using
    End Sub

    Private Sub LoadBoardsTxt()
        boardList.Clear()
        cmbBoards.Items.Clear()
        If Not File.Exists(boardsTxtPath) Then
            MessageBox.Show("boards.txt not found in project directory. File should be at: " & boardsTxtPath, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error)
            Return
        End If

        Try
            boardList = ArduinoBoard.ParseBoardsTxt(boardsTxtPath)
            For Each b In boardList
                cmbBoards.Items.Add(b.Name)
            Next
            If cmbBoards.Items.Count > 0 Then
                cmbBoards.SelectedIndex = 0
            End If
        Catch ex As Exception
            MessageBox.Show("Error loading boards.txt: " & ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error)
        End Try
    End Sub

    Private Sub DetectBoardList()
        If String.IsNullOrEmpty(arduinoCliPath) Then Return

        Try
            Dim psi As New ProcessStartInfo(arduinoCliPath, "board list")
            psi.RedirectStandardOutput = True
            psi.UseShellExecute = False
            psi.CreateNoWindow = True
            Dim p As Process = Process.Start(psi)
            Dim output = p.StandardOutput.ReadToEnd()
            p.WaitForExit()

            ' Log the output for debugging
            txtOutput.Text = "Detected boards: " & vbCrLf & output & vbCrLf
        Catch ex As Exception
            MessageBox.Show("Error detecting boards: " & ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error)
        End Try
    End Sub

    Private Function FindEsp32CorePath() As String
        ' Find ESP32 core directory in common locations
        Dim possibleLocations = New List(Of String) From {
            Path.Combine(arduinoUserDir, "packages", "esp32", "hardware", "esp32"),
            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Arduino15", "packages", "esp32", "hardware", "esp32"),
            Path.Combine(Application.StartupPath, "hardware", "esp32", "esp32"),
            Path.Combine(Path.GetDirectoryName(arduinoCliPath), "hardware", "esp32", "esp32")
        }

        ' Check each location
        For Each location As String In possibleLocations
            If Directory.Exists(location) Then
                ' Find the latest version directory
                Dim dirs() As String = Directory.GetDirectories(location)
                If dirs.Length > 0 Then
                    ' Sort by name (which is version) and get the latest
                    Array.Sort(dirs)
                    Return dirs(dirs.Length - 1)
                End If
            End If
        Next

        Return ""
    End Function

    Private Sub DetectEsp32Core()
        If String.IsNullOrEmpty(arduinoCliPath) Then Return

        Try
            ' First, check if we need to upgrade the core index
            Dim psi As New ProcessStartInfo(arduinoCliPath, "core update-index")
            psi.RedirectStandardOutput = True
            psi.UseShellExecute = False
            psi.CreateNoWindow = True
            Dim p As Process = Process.Start(psi)
            p.WaitForExit()

            ' Then check installed cores
            psi = New ProcessStartInfo(arduinoCliPath, "core list")
            psi.RedirectStandardOutput = True
            psi.UseShellExecute = False
            psi.CreateNoWindow = True
            p = Process.Start(psi)
            Dim output = p.StandardOutput.ReadToEnd()
            p.WaitForExit()

            ' Log the output for debugging
            txtOutput.AppendText("Detected cores: " & vbCrLf & output & vbCrLf)

            If Not output.ToLower().Contains("esp32") Then
                If MessageBox.Show("ESP32 core not found in arduino-cli. Would you like to install it now?", "ESP32 Core Not Found",
                                 MessageBoxButtons.YesNo, MessageBoxIcon.Question) = DialogResult.Yes Then
                    InstallEsp32Core()
                End If
            End If

            ' List the available boards
            psi = New ProcessStartInfo(arduinoCliPath, "board listall")
            psi.RedirectStandardOutput = True
            psi.UseShellExecute = False
            psi.CreateNoWindow = True
            p = Process.Start(psi)
            output = p.StandardOutput.ReadToEnd()
            p.WaitForExit()

            txtOutput.AppendText("Available boards: " & vbCrLf & output & vbCrLf)

            ' Look for ESP32 core path and check partition files
            Dim esp32CorePath = FindEsp32CorePath()
            If Not String.IsNullOrEmpty(esp32CorePath) Then
                txtOutput.AppendText($"Found ESP32 core at: {esp32CorePath}" & vbCrLf)

                ' Check if we have a huge_app partition file
                Dim partitionsDir = Path.Combine(esp32CorePath, "tools", "partitions")
                If Directory.Exists(partitionsDir) Then
                    Dim partitionFiles = Directory.GetFiles(partitionsDir, "*.csv")
                    txtOutput.AppendText($"Found {partitionFiles.Length} partition files in ESP32 core" & vbCrLf)

                    Dim hasHugeApp = False
                    For Each partFile In partitionFiles
                        If Path.GetFileName(partFile).ToLower().Contains("huge_app") Then
                            hasHugeApp = True
                            txtOutput.AppendText($"Found huge_app partition: {Path.GetFileName(partFile)}" & vbCrLf)
                            Exit For
                        End If
                    Next

                    If Not hasHugeApp Then
                        ' Create a huge app partition file
                        txtOutput.AppendText("No huge_app partition found. Creating one..." & vbCrLf)
                        CreateHugeAppPartition(partitionsDir)
                    End If
                End If
            End If
        Catch ex As Exception
            MessageBox.Show("Error checking ESP32 core: " & ex.Message)
        End Try
    End Sub

    Private Sub CreateHugeAppPartition(partitionsDir As String)
        ' Create a huge app partition file for 16MB flash
        Dim hugeAppPartitionFile = Path.Combine(partitionsDir, "huge_app.csv")

        ' Standard huge app partition for 16MB flash
        Dim partitionContent = "# Name,   Type, SubType, Offset,  Size, Flags" & vbCrLf &
                             "nvs,      data, nvs,     0x9000,  0x5000," & vbCrLf &
                             "otadata,  data, ota,     0xe000,  0x2000," & vbCrLf &
                             "app0,     app,  ota_0,   0x10000, 0xF00000," & vbCrLf &
                             "app1,     app,  ota_1,   0xF10000,0xF00000," & vbCrLf &
                             "spiffs,   data, spiffs,  0x1E10000,0x1F0000,"

        Try
            File.WriteAllText(hugeAppPartitionFile, partitionContent)
            txtOutput.AppendText($"Created huge_app partition file: {hugeAppPartitionFile}" & vbCrLf)

            ' Also modify boards.txt to add this partition scheme
            Dim esp32CorePath = Path.GetDirectoryName(partitionsDir)
            While Not Path.GetFileName(esp32CorePath).Equals("esp32")
                esp32CorePath = Path.GetDirectoryName(esp32CorePath)
            End While

            Dim boardsFile = Path.Combine(esp32CorePath, "boards.txt")
            If File.Exists(boardsFile) Then
                Dim boardsContent = File.ReadAllText(boardsFile)

                ' Check if the huge_app is already in boards.txt for ESP32P4
                If Not boardsContent.Contains("esp32p4.menu.PartitionScheme.huge_app") Then
                    Dim hugeAppConfig = vbCrLf &
                        "esp32p4.menu.PartitionScheme.huge_app=Huge App (16MB)" & vbCrLf &
                        "esp32p4.menu.PartitionScheme.huge_app.build.partitions=huge_app" & vbCrLf &
                        "esp32p4.menu.PartitionScheme.huge_app.upload.maximum_size=16777216" & vbCrLf

                    ' Add to the end of the file
                    File.AppendAllText(boardsFile, hugeAppConfig)
                    txtOutput.AppendText("Added huge_app partition scheme to ESP32P4 board configuration" & vbCrLf)
                End If
            End If
        Catch ex As Exception
            txtOutput.AppendText($"Error creating partition file: {ex.Message}" & vbCrLf)
        End Try
    End Sub

    Private Sub InstallEsp32Core()
        If String.IsNullOrEmpty(arduinoCliPath) Then Return

        txtOutput.AppendText("Installing ESP32 core..." & vbCrLf)

        Try
            Dim psi As New ProcessStartInfo(arduinoCliPath, "core update-index")
            psi.RedirectStandardOutput = True
            psi.UseShellExecute = False
            psi.CreateNoWindow = True

            Dim p As Process = Process.Start(psi)
            txtOutput.AppendText(p.StandardOutput.ReadToEnd() & vbCrLf)
            p.WaitForExit()

            psi = New ProcessStartInfo(arduinoCliPath, "core install esp32:esp32")
            psi.RedirectStandardOutput = True
            psi.UseShellExecute = False
            psi.CreateNoWindow = True

            p = Process.Start(psi)
            txtOutput.AppendText(p.StandardOutput.ReadToEnd() & vbCrLf)
            p.WaitForExit()

            txtOutput.AppendText("ESP32 core installation completed." & vbCrLf)
        Catch ex As Exception
            MessageBox.Show("Error installing ESP32 core: " & ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error)
        End Try
    End Sub

    Private Sub RefreshComPorts()
        Dim selectedPort = If(cmbComPorts.SelectedItem IsNot Nothing, cmbComPorts.SelectedItem.ToString(), "")

        cmbComPorts.Items.Clear()
        For Each port In SerialPort.GetPortNames()
            cmbComPorts.Items.Add(port)
        Next

        If cmbComPorts.Items.Count > 0 Then
            ' Try to reselect the previously selected port if it still exists
            Dim index = cmbComPorts.FindStringExact(selectedPort)
            If index >= 0 Then
                cmbComPorts.SelectedIndex = index
            Else
                cmbComPorts.SelectedIndex = 0
            End If
        End If
    End Sub

    Private Sub btnBoardConfig_Click(sender As Object, e As EventArgs) Handles btnBoardConfig.Click
        If cmbBoards.SelectedIndex = -1 Then
            MessageBox.Show("Select a board first.")
            Return
        End If
        Dim selectedBoard = boardList(cmbBoards.SelectedIndex)
        Dim frm As New BoardConfigForm(selectedBoard)
        If frm.ShowDialog() = DialogResult.OK Then
            boardConfig = frm.SelectedConfig
            ' Show the user that configuration has been saved
            txtOutput.AppendText("Board configuration saved: " & selectedBoard.Name & vbCrLf)
            For Each opt In boardConfig.Options
                ' Check if custom partition is selected - remind user to select a partition file
                If opt.Key.ToLower() = "partitionscheme" AndAlso opt.Value.ToLower() = "custom" Then
                    If String.IsNullOrEmpty(customPartitionFilePath) OrElse Not File.Exists(customPartitionFilePath) Then
                        AppendOutputLine("⚠️ You selected 'custom' partition scheme but no custom partition file is set.")
                        AppendOutputLine("Please select a custom partition file using the 'Select Partition' button.")
                    End If
                End If
                txtOutput.AppendText($"  - {opt.Key}: {opt.Value}" & vbCrLf)
            Next
        End If
    End Sub

    Private Sub btnCompile_Click(sender As Object, e As EventArgs) Handles btnCompile.Click
        If Not ReadyToBuild() Then Exit Sub
        pbCompile.Value = 0
        lblCompileProgress.Text = "Compile Progress: 0%"
        btnCompile.Enabled = False
        btnGenBin.Enabled = False
        btnUpload.Enabled = False
        txtOutput.Text = "Compiling in progress..." & vbCrLf

        Dim selectedBoardIndex As Integer = cmbBoards.SelectedIndex
        Dim selectedComPort As String = If(cmbComPorts.SelectedItem Is Nothing, "", cmbComPorts.SelectedItem.ToString())
        Dim configCopy As ArduinoBoardConfig = boardConfig

        bgwCompile.RunWorkerAsync(New CompileArgs With {
            .SelectedBoardIndex = selectedBoardIndex,
            .SelectedComPort = selectedComPort,
            .ConfigCopy = configCopy,
            .IsBin = False
        })
    End Sub

    Private Sub btnGenBin_Click(sender As Object, e As EventArgs) Handles btnGenBin.Click
        If Not ReadyToBuild() Then Exit Sub
        pbCompile.Value = 0
        lblCompileProgress.Text = "Compile Progress: 0%"
        btnCompile.Enabled = False
        btnGenBin.Enabled = False
        btnUpload.Enabled = False
        txtOutput.Text = "Binary generation in progress..." & vbCrLf

        Dim selectedBoardIndex As Integer = cmbBoards.SelectedIndex
        Dim selectedComPort As String = If(cmbComPorts.SelectedItem Is Nothing, "", cmbComPorts.SelectedItem.ToString())
        Dim configCopy As ArduinoBoardConfig = boardConfig

        bgwCompile.RunWorkerAsync(New CompileArgs With {
            .SelectedBoardIndex = selectedBoardIndex,
            .SelectedComPort = selectedComPort,
            .ConfigCopy = configCopy,
            .IsBin = True
        })
    End Sub

    Private Sub btnUpload_Click(sender As Object, e As EventArgs) Handles btnUpload.Click
        If Not ReadyToBuild() Then Exit Sub
        pbUpload.Value = 0
        lblUploadProgress.Text = "Upload Progress: 0%"
        btnCompile.Enabled = False
        btnGenBin.Enabled = False
        btnUpload.Enabled = False
        txtOutput.Text = "Upload in progress..." & vbCrLf

        Dim selectedBoardIndex As Integer = cmbBoards.SelectedIndex
        Dim selectedComPort As String = If(cmbComPorts.SelectedItem Is Nothing, "", cmbComPorts.SelectedItem.ToString())
        Dim configCopy As ArduinoBoardConfig = boardConfig

        bgwUpload.RunWorkerAsync(New CompileArgs With {
            .SelectedBoardIndex = selectedBoardIndex,
            .SelectedComPort = selectedComPort,
            .ConfigCopy = configCopy,
            .IsBin = False
        })
    End Sub

    Private Function ReadyToBuild() As Boolean
        If String.IsNullOrEmpty(arduinoCliPath) OrElse Not File.Exists(arduinoCliPath) Then
            MessageBox.Show("Please select a valid arduino-cli.exe.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Return False
        End If
        If String.IsNullOrEmpty(sketchPath) OrElse Not Directory.Exists(sketchPath) Then
            MessageBox.Show("Please select a valid sketch folder.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Return False
        End If

        ' Check if sketch has .ino file
        Dim inoFiles = Directory.GetFiles(sketchPath, "*.ino")
        If inoFiles.Length = 0 Then
            MessageBox.Show("The selected folder does not contain an .ino sketch file.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Return False
        End If

        If cmbBoards.SelectedIndex = -1 Then
            MessageBox.Show("Please select a board.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Return False
        End If
        If cmbComPorts.SelectedIndex = -1 Then
            MessageBox.Show("Please select a COM port.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            Return False
        End If
        If boardConfig Is Nothing Then
            MessageBox.Show("Please configure and save the board settings (Board Config).", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning)
            btnBoardConfig.PerformClick()
            If boardConfig Is Nothing Then Return False
        End If

        ' Check if custom partition is selected but no file is set
        Dim hasCustomPartition = False
        If boardConfig IsNot Nothing Then
            For Each opt In boardConfig.Options
                If opt.Key.ToLower() = "partitionscheme" AndAlso opt.Value.ToLower() = "custom" Then
                    hasCustomPartition = True
                    Exit For
                End If
            Next
        End If

        If hasCustomPartition AndAlso (String.IsNullOrEmpty(customPartitionFilePath) OrElse Not File.Exists(customPartitionFilePath)) Then
            Dim result = MessageBox.Show("You selected 'custom' partition scheme but no custom partition file is set." & vbCrLf &
                                      "Do you want to select a partition file now?",
                                     "Custom Partition Required", MessageBoxButtons.YesNo, MessageBoxIcon.Warning)
            If result = DialogResult.Yes Then
                btnSelectPartition.PerformClick()
                If String.IsNullOrEmpty(customPartitionFilePath) OrElse Not File.Exists(customPartitionFilePath) Then
                    Return False
                End If
            End If
        End If

        Return True
    End Function

    Private Class CompileArgs
        Public Property SelectedBoardIndex As Integer
        Public Property SelectedComPort As String
        Public Property ConfigCopy As ArduinoBoardConfig
        Public Property IsBin As Boolean
    End Class

    ' --- BackgroundWorker Compile ---
    Private Sub bgwCompile_DoWork(sender As Object, e As DoWorkEventArgs) Handles bgwCompile.DoWork
        Dim args = CType(e.Argument, CompileArgs)
        e.Result = RunArduinoCliWithRealtimeProgress(
            If(args.IsBin, "compile --export-binaries", "compile"),
            CType(sender, BackgroundWorker),
            args.SelectedBoardIndex,
            args.SelectedComPort,
            args.ConfigCopy,
            cliStagesCompile
        )
    End Sub

    Private Sub bgwCompile_ProgressChanged(sender As Object, e As ProgressChangedEventArgs) Handles bgwCompile.ProgressChanged
        pbCompile.Value = Math.Min(e.ProgressPercentage, 100)
        lblCompileProgress.Text = $"Compile Progress: {pbCompile.Value}%"
    End Sub

    Private Sub bgwCompile_RunWorkerCompleted(sender As Object, e As RunWorkerCompletedEventArgs) Handles bgwCompile.RunWorkerCompleted
        pbCompile.Value = 100
        lblCompileProgress.Text = "Compile Progress: 100%"
        btnCompile.Enabled = True
        btnGenBin.Enabled = True
        btnUpload.Enabled = True
        If e.Error IsNot Nothing Then
            txtOutput.Text &= vbCrLf & "Error: " & e.Error.Message
        ElseIf e.Result IsNot Nothing Then
            txtOutput.Text &= e.Result.ToString()
        End If

        AppendOutputLine("Compilation completed.")
    End Sub

    ' --- BackgroundWorker Upload ---
    Private Sub bgwUpload_DoWork(sender As Object, e As DoWorkEventArgs) Handles bgwUpload.DoWork
        Dim args = CType(e.Argument, CompileArgs)
        e.Result = RunArduinoCliWithRealtimeProgress(
            "upload",
            CType(sender, BackgroundWorker),
            args.SelectedBoardIndex,
            args.SelectedComPort,
            args.ConfigCopy,
            cliStagesUpload
        )
    End Sub

    Private Sub bgwUpload_ProgressChanged(sender As Object, e As ProgressChangedEventArgs) Handles bgwUpload.ProgressChanged
        pbUpload.Value = Math.Min(e.ProgressPercentage, 100)
        lblUploadProgress.Text = $"Upload Progress: {pbUpload.Value}%"
    End Sub

    Private Sub bgwUpload_RunWorkerCompleted(sender As Object, e As RunWorkerCompletedEventArgs) Handles bgwUpload.RunWorkerCompleted
        pbUpload.Value = 100
        lblUploadProgress.Text = "Upload Progress: 100%"
        btnCompile.Enabled = True
        btnGenBin.Enabled = True
        btnUpload.Enabled = True
        If e.Error IsNot Nothing Then
            txtOutput.Text &= vbCrLf & "Error: " & e.Error.Message
        ElseIf e.Result IsNot Nothing Then
            txtOutput.Text &= e.Result.ToString()
        End If

        AppendOutputLine("Upload completed.")
    End Sub

    ' --- Core CLI handler with smooth real-time progress ---
    Private Function RunArduinoCliWithRealtimeProgress(command As String, bgw As BackgroundWorker, selectedBoardIndex As Integer, selectedComPort As String, configCopy As ArduinoBoardConfig, stages() As String) As String
        Dim selectedBoard = boardList(selectedBoardIndex)
        Dim baseFqbn = selectedBoard.FQBN
        Dim outputText As New StringBuilder()

        ' Build command properly with board options
        ' Log options for debugging
        If configCopy IsNot Nothing AndAlso configCopy.Options.Count > 0 Then
            AppendOutputLine("Board options:")
            For Each opt In configCopy.Options
                AppendOutputLine($"  - {opt.Key}: {opt.Value}")
            Next
        End If

        ' Check if we're dealing with custom partition
        Dim isEsp32P4 As Boolean = baseFqbn.Contains("esp32p4")
        Dim hasCustomPartition As Boolean = False
        Dim partitionScheme As String = ""

        If configCopy IsNot Nothing Then
            For Each opt In configCopy.Options
                If opt.Key.ToLower() = "partitionscheme" Then
                    partitionScheme = opt.Value.ToLower()
                    If partitionScheme = "custom" Then
                        hasCustomPartition = True
                    End If
                    Exit For
                End If
            Next
        End If

        ' For custom partition, we need special handling
        Dim hugeAppPartitionPath As String = ""
        Dim backupPartitionPath As String = ""

        ' Process custom partition if needed
        If hasCustomPartition AndAlso Not String.IsNullOrEmpty(customPartitionFilePath) AndAlso File.Exists(customPartitionFilePath) Then
            Dim esp32CorePath As String = FindEsp32CorePath()
            If Not String.IsNullOrEmpty(esp32CorePath) Then
                AppendOutputLine($"Found ESP32 core at: {esp32CorePath}")

                ' Prepare custom partition - copy to ESP32 core's huge_app.csv
                Dim partitionsDir = Path.Combine(esp32CorePath, "tools", "partitions")
                hugeAppPartitionPath = Path.Combine(partitionsDir, "huge_app.csv")
                backupPartitionPath = hugeAppPartitionPath & ".bak"

                If Directory.Exists(partitionsDir) Then
                    Try
                        ' Backup original huge_app.csv if it exists
                        If File.Exists(hugeAppPartitionPath) AndAlso Not File.Exists(backupPartitionPath) Then
                            File.Copy(hugeAppPartitionPath, backupPartitionPath, True)
                            AppendOutputLine("Backed up original huge_app.csv")
                        End If

                        ' Copy our custom partition file to huge_app.csv
                        File.Copy(customPartitionFilePath, hugeAppPartitionPath, True)
                        AppendOutputLine($"Copied custom partition to ESP32 core: {hugeAppPartitionPath}")

                        ' Change partition scheme in board config to huge_app
                        If configCopy IsNot Nothing Then
                            Dim keys = configCopy.Options.Keys.ToList()
                            For i As Integer = 0 To keys.Count - 1
                                If keys(i).ToLower() = "partitionscheme" Then
                                    configCopy.Options(keys(i)) = "huge_app"
                                    AppendOutputLine("Changed partition scheme to huge_app in FQBN")
                                    Exit For
                                End If
                            Next
                        End If

                        ' Also create build.json in sketch directory to force 16MB partition
                        Dim buildJsonFile = Path.Combine(sketchPath, "build.json")
                        Dim buildJsonContent = "{" & vbCrLf &
                                         "  ""build.partitions"": ""huge_app""," & vbCrLf &
                                         "  ""build.flash_size"": ""16MB""," & vbCrLf &
                                         "  ""upload.maximum_size"": 16777216" & vbCrLf &
                                         "}"

                        File.WriteAllText(buildJsonFile, buildJsonContent)
                        AppendOutputLine("Created build.json in sketch directory to force 16MB partition")

                    Catch ex As Exception
                        AppendOutputLine("Error handling custom partition: " & ex.Message)
                    End Try
                End If
            End If
        End If

        ' Build FQBN string based on board and options
        Dim fullFqbn As String = baseFqbn

        ' For ESP32P4 with custom partition, use simplified FQBN with huge_app
        If isEsp32P4 AndAlso hasCustomPartition Then
            fullFqbn = baseFqbn & ":PartitionScheme=huge_app"
            AppendOutputLine($"Using simplified FQBN with huge_app: {fullFqbn}")
        Else
            ' For other cases, use full FQBN with all options
            If configCopy IsNot Nothing AndAlso configCopy.Options.Count > 0 Then
                Dim optionsString As String = ""
                For Each opt In configCopy.Options
                    optionsString &= ":" & opt.Key & "=" & opt.Value
                Next
                fullFqbn &= optionsString
            End If
            AppendOutputLine($"Using FQBN with options: {fullFqbn}")
        End If

        ' Build CLI command
        Dim args As String
        If command.Contains("upload") Then
            args = $"{command} -p {selectedComPort} -b {fullFqbn} ""{sketchPath}"""
        Else
            args = $"{command} -b {fullFqbn} ""{sketchPath}"""
        End If

        AppendOutputLine("Running command: " & arduinoCliPath & " " & args)

        ' Setup process
        Dim psi As New ProcessStartInfo(arduinoCliPath, args) With {
            .RedirectStandardOutput = True,
            .RedirectStandardError = True,
            .UseShellExecute = False,
            .CreateNoWindow = True,
            .WorkingDirectory = Path.GetDirectoryName(sketchPath)
        }

        ' Create a process and get ready to read output
        Dim proc As New Process() With {.StartInfo = psi, .EnableRaisingEvents = True}

        ' Setup an event handler for output and error
        Dim currentStage As Integer = 0
        Dim progress As Integer = 5 ' Start at 5%
        Dim lastReportedProgress As Integer = 0
        Dim outputCompleted As Boolean = False
        Dim errorCompleted As Boolean = False
        Dim foundCorrectPartitionSize As Boolean = False

        ' Event handlers for real-time output
        AddHandler proc.OutputDataReceived, Sub(sender, e)
                                                If e.Data IsNot Nothing Then
                                                    outputText.AppendLine(e.Data)
                                                    AppendOutputLine(e.Data)

                                                    ' Check for 16MB partition size in output
                                                    If e.Data.Contains("Maximum is 16777216 bytes") Then
                                                        AppendOutputLine("✅ Successfully using 16MB partition scheme!")
                                                        foundCorrectPartitionSize = True
                                                    End If

                                                    ' Check for stage completion
                                                    For i = 0 To stages.Length - 1
                                                        If e.Data.IndexOf(stages(i), StringComparison.OrdinalIgnoreCase) >= 0 Then
                                                            Dim newProgress = ((i + 1) * 90) / stages.Length
                                                            If newProgress > progress Then
                                                                progress = newProgress
                                                                If bgw IsNot Nothing AndAlso bgw.WorkerReportsProgress Then
                                                                    bgw.ReportProgress(progress)
                                                                    lastReportedProgress = progress
                                                                End If
                                                            End If
                                                            Exit For
                                                        End If
                                                    Next

                                                    ' Force periodic progress updates even without specific stage marker
                                                    If Not e.Data.Trim().Equals(String.Empty) Then
                                                        progress = Math.Min(95, progress + 1)
                                                        If progress > lastReportedProgress Then
                                                            If bgw IsNot Nothing AndAlso bgw.WorkerReportsProgress Then
                                                                bgw.ReportProgress(progress)
                                                                lastReportedProgress = progress
                                                            End If
                                                        End If
                                                    End If
                                                Else
                                                    outputCompleted = True
                                                End If
                                            End Sub

        AddHandler proc.ErrorDataReceived, Sub(sender, e)
                                               If e.Data IsNot Nothing Then
                                                   outputText.AppendLine(e.Data)
                                                   AppendOutputLine(e.Data)
                                               Else
                                                   errorCompleted = True
                                               End If
                                           End Sub

        ' Start the process and read output asynchronously
        proc.Start()
        proc.BeginOutputReadLine()
        proc.BeginErrorReadLine()

        ' Report initial progress
        bgw.ReportProgress(5)

        ' Wait for process to complete with periodic UI updates
        Dim startTime = DateTime.Now
        While Not proc.HasExited OrElse Not outputCompleted OrElse Not errorCompleted
            ' Check for cancellation
            If bgw.CancellationPending Then
                Try
                    ' Attempt to kill the process
                    If Not proc.HasExited Then
                        proc.Kill()
                    End If
                Catch ex As Exception
                    ' Ignore errors on killing the process
                End Try
                Return "Operation was canceled."
            End If

            ' Periodically update progress if there's been a long time without updates
            If (DateTime.Now - startTime).TotalSeconds > 5 AndAlso lastReportedProgress < 90 Then
                progress = Math.Min(90, lastReportedProgress + 5)
                bgw.ReportProgress(progress)
                lastReportedProgress = progress
                startTime = DateTime.Now
            End If

            Thread.Sleep(100)  ' Don't burn CPU cycles
        End While

        ' Clean up
        RemoveHandler proc.OutputDataReceived, Nothing
        RemoveHandler proc.ErrorDataReceived, Nothing

        ' Report final status
        bgw.ReportProgress(100)

        ' Clean up custom partition files if needed
        If hasCustomPartition AndAlso Not String.IsNullOrEmpty(hugeAppPartitionPath) AndAlso File.Exists(backupPartitionPath) Then
            Try
                ' Restore original huge_app.csv
                File.Copy(backupPartitionPath, hugeAppPartitionPath, True)
                File.Delete(backupPartitionPath)
                AppendOutputLine("Restored original huge_app.csv")
            Catch ex As Exception
                AppendOutputLine("Warning: Could not restore original partition file: " & ex.Message)
            End Try
        End If

        ' Check if we succeeded with 16MB partition
        If hasCustomPartition AndAlso Not foundCorrectPartitionSize Then
            AppendOutputLine("⚠️ Warning: Not using 16MB partition scheme. Will try to improve for next compile.")
        End If

        ' Return result
        Return outputText.ToString()
    End Function

    Private Sub AppendOutputLine(line As String)
        If txtOutput.InvokeRequired Then
            txtOutput.Invoke(Sub()
                                 txtOutput.AppendText(line & vbCrLf)
                                 txtOutput.SelectionStart = txtOutput.TextLength
                                 txtOutput.ScrollToCaret()
                             End Sub)
        Else
            txtOutput.AppendText(line & vbCrLf)
            txtOutput.SelectionStart = txtOutput.TextLength
            txtOutput.ScrollToCaret()
        End If
    End Sub
End Class