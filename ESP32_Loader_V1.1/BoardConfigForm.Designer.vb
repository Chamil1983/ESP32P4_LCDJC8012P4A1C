<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()>
Partial Class BoardConfigForm
    Inherits System.Windows.Forms.Form

    Friend WithEvents tblConfig As TableLayoutPanel
    Friend WithEvents btnOK As Button
    Friend WithEvents btnCancel As Button
    Friend WithEvents btnSave As Button
    Friend WithEvents pnlButtons As Panel

    Private Sub InitializeComponent()
        Me.tblConfig = New System.Windows.Forms.TableLayoutPanel()
        Me.pnlButtons = New System.Windows.Forms.Panel()
        Me.btnSave = New System.Windows.Forms.Button()
        Me.btnOK = New System.Windows.Forms.Button()
        Me.btnCancel = New System.Windows.Forms.Button()
        Me.pnlButtons.SuspendLayout()
        Me.SuspendLayout()
        '
        'tblConfig
        '
        Me.tblConfig.Anchor = CType(((System.Windows.Forms.AnchorStyles.Top Or System.Windows.Forms.AnchorStyles.Left) _
            Or System.Windows.Forms.AnchorStyles.Right), System.Windows.Forms.AnchorStyles)
        Me.tblConfig.AutoSize = True
        Me.tblConfig.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink
        Me.tblConfig.ColumnCount = 2
        Me.tblConfig.ColumnStyles.Add(New System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 40.0!))
        Me.tblConfig.ColumnStyles.Add(New System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 60.0!))
        Me.tblConfig.Location = New System.Drawing.Point(12, 12)
        Me.tblConfig.Name = "tblConfig"
        Me.tblConfig.RowCount = 1
        Me.tblConfig.RowStyles.Add(New System.Windows.Forms.RowStyle())
        Me.tblConfig.Size = New System.Drawing.Size(360, 0)
        Me.tblConfig.TabIndex = 0
        '
        'pnlButtons
        '
        Me.pnlButtons.Anchor = CType(((System.Windows.Forms.AnchorStyles.Bottom Or System.Windows.Forms.AnchorStyles.Left) _
            Or System.Windows.Forms.AnchorStyles.Right), System.Windows.Forms.AnchorStyles)
        Me.pnlButtons.BackColor = System.Drawing.SystemColors.Control
        Me.pnlButtons.Controls.Add(Me.btnSave)
        Me.pnlButtons.Controls.Add(Me.btnOK)
        Me.pnlButtons.Controls.Add(Me.btnCancel)
        Me.pnlButtons.Location = New System.Drawing.Point(4, 543)
        Me.pnlButtons.Name = "pnlButtons"
        Me.pnlButtons.Size = New System.Drawing.Size(384, 45)
        Me.pnlButtons.TabIndex = 1
        '
        'btnSave
        '
        Me.btnSave.Anchor = System.Windows.Forms.AnchorStyles.Right
        Me.btnSave.Location = New System.Drawing.Point(80, 10)
        Me.btnSave.Name = "btnSave"
        Me.btnSave.Size = New System.Drawing.Size(75, 28)
        Me.btnSave.TabIndex = 2
        Me.btnSave.Text = "Save"
        Me.btnSave.UseVisualStyleBackColor = True
        '
        'btnOK
        '
        Me.btnOK.Anchor = System.Windows.Forms.AnchorStyles.Right
        Me.btnOK.Location = New System.Drawing.Point(170, 10)
        Me.btnOK.Name = "btnOK"
        Me.btnOK.Size = New System.Drawing.Size(75, 28)
        Me.btnOK.TabIndex = 3
        Me.btnOK.Text = "OK && Close"
        Me.btnOK.UseVisualStyleBackColor = True
        '
        'btnCancel
        '
        Me.btnCancel.Anchor = System.Windows.Forms.AnchorStyles.Right
        Me.btnCancel.Location = New System.Drawing.Point(260, 10)
        Me.btnCancel.Name = "btnCancel"
        Me.btnCancel.Size = New System.Drawing.Size(75, 28)
        Me.btnCancel.TabIndex = 4
        Me.btnCancel.Text = "Cancel"
        Me.btnCancel.UseVisualStyleBackColor = True
        '
        'BoardConfigForm
        '
        Me.ClientSize = New System.Drawing.Size(400, 600)
        Me.Controls.Add(Me.tblConfig)
        Me.Controls.Add(Me.pnlButtons)
        Me.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog
        Me.MaximizeBox = False
        Me.MinimumSize = New System.Drawing.Size(400, 320)
        Me.Name = "BoardConfigForm"
        Me.StartPosition = System.Windows.Forms.FormStartPosition.CenterParent
        Me.Text = "Board Configuration"
        Me.pnlButtons.ResumeLayout(False)
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
End Class