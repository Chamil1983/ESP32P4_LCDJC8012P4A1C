<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()>
Partial Class MainForm
    Inherits System.Windows.Forms.Form

    Friend WithEvents btnSelectCli As Button
    Friend WithEvents btnSelectSketch As Button
    Friend WithEvents btnBoardConfig As Button
    Friend WithEvents btnCompile As Button
    Friend WithEvents btnGenBin As Button
    Friend WithEvents btnUpload As Button
    Friend WithEvents cmbBoards As ComboBox
    Friend WithEvents cmbComPorts As ComboBox
    Friend WithEvents txtCliPath As TextBox
    Friend WithEvents txtSketchPath As TextBox
    Friend WithEvents txtOutput As TextBox
    Friend WithEvents lblCompileProgress As Label
    Friend WithEvents lblUploadProgress As Label
    Friend WithEvents pbCompile As ProgressBar
    Friend WithEvents pbUpload As ProgressBar
    Friend WithEvents btnRefreshPorts As Button

    Private Sub InitializeComponent()
        Me.btnSelectCli = New System.Windows.Forms.Button()
        Me.btnSelectSketch = New System.Windows.Forms.Button()
        Me.btnBoardConfig = New System.Windows.Forms.Button()
        Me.btnCompile = New System.Windows.Forms.Button()
        Me.btnGenBin = New System.Windows.Forms.Button()
        Me.btnUpload = New System.Windows.Forms.Button()
        Me.cmbBoards = New System.Windows.Forms.ComboBox()
        Me.cmbComPorts = New System.Windows.Forms.ComboBox()
        Me.txtCliPath = New System.Windows.Forms.TextBox()
        Me.txtSketchPath = New System.Windows.Forms.TextBox()
        Me.txtOutput = New System.Windows.Forms.TextBox()
        Me.lblCompileProgress = New System.Windows.Forms.Label()
        Me.lblUploadProgress = New System.Windows.Forms.Label()
        Me.pbCompile = New System.Windows.Forms.ProgressBar()
        Me.pbUpload = New System.Windows.Forms.ProgressBar()
        Me.btnRefreshPorts = New System.Windows.Forms.Button()
        Me.txtPartitionPath = New System.Windows.Forms.TextBox()
        Me.btnSelectPartition = New System.Windows.Forms.Button()
        Me.SuspendLayout()
        '
        'btnSelectCli
        '
        Me.btnSelectCli.Location = New System.Drawing.Point(12, 12)
        Me.btnSelectCli.Name = "btnSelectCli"
        Me.btnSelectCli.Size = New System.Drawing.Size(140, 23)
        Me.btnSelectCli.TabIndex = 0
        Me.btnSelectCli.Text = "Select arduino-cli.exe"
        Me.btnSelectCli.UseVisualStyleBackColor = True
        '
        'btnSelectSketch
        '
        Me.btnSelectSketch.Location = New System.Drawing.Point(12, 45)
        Me.btnSelectSketch.Name = "btnSelectSketch"
        Me.btnSelectSketch.Size = New System.Drawing.Size(140, 23)
        Me.btnSelectSketch.TabIndex = 2
        Me.btnSelectSketch.Text = "Select Sketch Folder"
        Me.btnSelectSketch.UseVisualStyleBackColor = True
        '
        'btnBoardConfig
        '
        Me.btnBoardConfig.Location = New System.Drawing.Point(440, 136)
        Me.btnBoardConfig.Name = "btnBoardConfig"
        Me.btnBoardConfig.Size = New System.Drawing.Size(80, 23)
        Me.btnBoardConfig.TabIndex = 5
        Me.btnBoardConfig.Text = "Board Config"
        Me.btnBoardConfig.UseVisualStyleBackColor = True
        '
        'btnCompile
        '
        Me.btnCompile.Location = New System.Drawing.Point(12, 208)
        Me.btnCompile.Name = "btnCompile"
        Me.btnCompile.Size = New System.Drawing.Size(75, 25)
        Me.btnCompile.TabIndex = 8
        Me.btnCompile.Text = "Compile"
        Me.btnCompile.UseVisualStyleBackColor = True
        '
        'btnGenBin
        '
        Me.btnGenBin.Location = New System.Drawing.Point(100, 208)
        Me.btnGenBin.Name = "btnGenBin"
        Me.btnGenBin.Size = New System.Drawing.Size(120, 25)
        Me.btnGenBin.TabIndex = 9
        Me.btnGenBin.Text = "Generate Binary"
        Me.btnGenBin.UseVisualStyleBackColor = True
        '
        'btnUpload
        '
        Me.btnUpload.Location = New System.Drawing.Point(230, 208)
        Me.btnUpload.Name = "btnUpload"
        Me.btnUpload.Size = New System.Drawing.Size(75, 25)
        Me.btnUpload.TabIndex = 10
        Me.btnUpload.Text = "Upload"
        Me.btnUpload.UseVisualStyleBackColor = True
        '
        'cmbBoards
        '
        Me.cmbBoards.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList
        Me.cmbBoards.FormattingEnabled = True
        Me.cmbBoards.Location = New System.Drawing.Point(170, 138)
        Me.cmbBoards.Name = "cmbBoards"
        Me.cmbBoards.Size = New System.Drawing.Size(250, 21)
        Me.cmbBoards.TabIndex = 4
        '
        'cmbComPorts
        '
        Me.cmbComPorts.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList
        Me.cmbComPorts.FormattingEnabled = True
        Me.cmbComPorts.Location = New System.Drawing.Point(170, 171)
        Me.cmbComPorts.Name = "cmbComPorts"
        Me.cmbComPorts.Size = New System.Drawing.Size(120, 21)
        Me.cmbComPorts.TabIndex = 6
        '
        'txtCliPath
        '
        Me.txtCliPath.Location = New System.Drawing.Point(170, 14)
        Me.txtCliPath.Name = "txtCliPath"
        Me.txtCliPath.ReadOnly = True
        Me.txtCliPath.Size = New System.Drawing.Size(350, 20)
        Me.txtCliPath.TabIndex = 1
        '
        'txtSketchPath
        '
        Me.txtSketchPath.Location = New System.Drawing.Point(170, 47)
        Me.txtSketchPath.Name = "txtSketchPath"
        Me.txtSketchPath.ReadOnly = True
        Me.txtSketchPath.Size = New System.Drawing.Size(350, 20)
        Me.txtSketchPath.TabIndex = 3
        '
        'txtOutput
        '
        Me.txtOutput.Location = New System.Drawing.Point(12, 303)
        Me.txtOutput.Multiline = True
        Me.txtOutput.Name = "txtOutput"
        Me.txtOutput.ReadOnly = True
        Me.txtOutput.ScrollBars = System.Windows.Forms.ScrollBars.Both
        Me.txtOutput.Size = New System.Drawing.Size(508, 120)
        Me.txtOutput.TabIndex = 10
        '
        'lblCompileProgress
        '
        Me.lblCompileProgress.AutoSize = True
        Me.lblCompileProgress.Location = New System.Drawing.Point(12, 243)
        Me.lblCompileProgress.Name = "lblCompileProgress"
        Me.lblCompileProgress.Size = New System.Drawing.Size(91, 13)
        Me.lblCompileProgress.TabIndex = 11
        Me.lblCompileProgress.Text = "Compile Progress:"
        '
        'lblUploadProgress
        '
        Me.lblUploadProgress.AutoSize = True
        Me.lblUploadProgress.Location = New System.Drawing.Point(12, 273)
        Me.lblUploadProgress.Name = "lblUploadProgress"
        Me.lblUploadProgress.Size = New System.Drawing.Size(88, 13)
        Me.lblUploadProgress.TabIndex = 13
        Me.lblUploadProgress.Text = "Upload Progress:"
        '
        'pbCompile
        '
        Me.pbCompile.Location = New System.Drawing.Point(120, 238)
        Me.pbCompile.Name = "pbCompile"
        Me.pbCompile.Size = New System.Drawing.Size(400, 22)
        Me.pbCompile.TabIndex = 12
        '
        'pbUpload
        '
        Me.pbUpload.Location = New System.Drawing.Point(120, 268)
        Me.pbUpload.Name = "pbUpload"
        Me.pbUpload.Size = New System.Drawing.Size(400, 22)
        Me.pbUpload.TabIndex = 14
        '
        'btnRefreshPorts
        '
        Me.btnRefreshPorts.Location = New System.Drawing.Point(300, 170)
        Me.btnRefreshPorts.Name = "btnRefreshPorts"
        Me.btnRefreshPorts.Size = New System.Drawing.Size(100, 23)
        Me.btnRefreshPorts.TabIndex = 7
        Me.btnRefreshPorts.Text = "Refresh Ports"
        Me.btnRefreshPorts.UseVisualStyleBackColor = True
        '
        'txtPartitionPath
        '
        Me.txtPartitionPath.Location = New System.Drawing.Point(170, 85)
        Me.txtPartitionPath.Name = "txtPartitionPath"
        Me.txtPartitionPath.ReadOnly = True
        Me.txtPartitionPath.Size = New System.Drawing.Size(350, 20)
        Me.txtPartitionPath.TabIndex = 16
        '
        'btnSelectPartition
        '
        Me.btnSelectPartition.Location = New System.Drawing.Point(12, 83)
        Me.btnSelectPartition.Name = "btnSelectPartition"
        Me.btnSelectPartition.Size = New System.Drawing.Size(140, 23)
        Me.btnSelectPartition.TabIndex = 15
        Me.btnSelectPartition.Text = "Select Partition"
        Me.btnSelectPartition.UseVisualStyleBackColor = True
        '
        'MainForm
        '
        Me.ClientSize = New System.Drawing.Size(534, 434)
        Me.Controls.Add(Me.txtPartitionPath)
        Me.Controls.Add(Me.btnSelectPartition)
        Me.Controls.Add(Me.lblCompileProgress)
        Me.Controls.Add(Me.pbCompile)
        Me.Controls.Add(Me.lblUploadProgress)
        Me.Controls.Add(Me.pbUpload)
        Me.Controls.Add(Me.txtOutput)
        Me.Controls.Add(Me.btnUpload)
        Me.Controls.Add(Me.btnGenBin)
        Me.Controls.Add(Me.btnCompile)
        Me.Controls.Add(Me.btnRefreshPorts)
        Me.Controls.Add(Me.cmbComPorts)
        Me.Controls.Add(Me.btnBoardConfig)
        Me.Controls.Add(Me.cmbBoards)
        Me.Controls.Add(Me.txtSketchPath)
        Me.Controls.Add(Me.btnSelectSketch)
        Me.Controls.Add(Me.txtCliPath)
        Me.Controls.Add(Me.btnSelectCli)
        Me.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog
        Me.MaximizeBox = False
        Me.Name = "MainForm"
        Me.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen
        Me.Text = "ESP32 Sketch Uploader"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub

    Friend WithEvents txtPartitionPath As TextBox
    Friend WithEvents btnSelectPartition As Button
End Class