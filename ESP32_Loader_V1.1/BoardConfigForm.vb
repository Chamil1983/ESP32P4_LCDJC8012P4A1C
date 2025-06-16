Public Class BoardConfigForm
    Public Property SelectedConfig As ArduinoBoardConfig
    Public Property Saved As Boolean = False

    Private board As ArduinoBoard

    Public Sub New(selectedBoard As ArduinoBoard)
        InitializeComponent()
        board = selectedBoard
        Me.Text = $"Board Configuration - {board.Name}"
        LoadConfigs()
    End Sub

    Private Sub LoadConfigs()
        tblConfig.Controls.Clear()
        tblConfig.RowStyles.Clear()
        tblConfig.RowCount = 0
        Dim rowIndex As Integer = 0

        For Each kvp In board.ConfigOptions
            Dim optionType = kvp.Key
            Dim lbl As New Label With {
                .Text = optionType,
                .AutoSize = True,
                .Anchor = AnchorStyles.Left,
                .TextAlign = ContentAlignment.MiddleLeft
            }

            Dim cmb As New ComboBox With {
                .Name = "cmb" & optionType,
                .DropDownStyle = ComboBoxStyle.DropDownList,
                .Anchor = AnchorStyles.Left Or AnchorStyles.Right,
                .Width = 180,
                .Tag = optionType  ' Store the option type
            }

            For Each optionPair In kvp.Value
                ' Split the option value and display name
                Dim parts = optionPair.Split("="c)
                If parts.Length >= 2 Then
                    Dim value = parts(0)
                    Dim display = String.Join("=", parts.Skip(1))
                    Dim item As New KeyValuePair(Of String, String)(value, display)
                    cmb.Items.Add(item)
                    cmb.DisplayMember = "Value" ' Display the human-readable part
                    cmb.ValueMember = "Key"    ' Store the configuration value
                End If
            Next

            If cmb.Items.Count > 0 Then
                cmb.SelectedIndex = 0
            End If

            tblConfig.RowCount += 1
            tblConfig.RowStyles.Add(New RowStyle(SizeType.AutoSize))
            tblConfig.Controls.Add(lbl, 0, rowIndex)
            tblConfig.Controls.Add(cmb, 1, rowIndex)
            rowIndex += 1
        Next
    End Sub

    Private Sub btnSave_Click(sender As Object, e As EventArgs) Handles btnSave.Click
        Dim config As New ArduinoBoardConfig()

        For Each ctrl In tblConfig.Controls
            If TypeOf ctrl Is ComboBox Then
                Dim cmb = DirectCast(ctrl, ComboBox)
                If cmb.SelectedIndex >= 0 AndAlso cmb.Tag IsNot Nothing Then
                    Dim optionType = cmb.Tag.ToString()
                    Dim selectedItem = DirectCast(cmb.SelectedItem, KeyValuePair(Of String, String))
                    config.Options(optionType) = selectedItem.Key ' Use the configuration value, not display text
                End If
            End If
        Next

        SelectedConfig = config
        Saved = True
        MessageBox.Show("Configuration saved.", "Success", MessageBoxButtons.OK, MessageBoxIcon.Information)
    End Sub

    Private Sub btnOK_Click(sender As Object, e As EventArgs) Handles btnOK.Click
        If Not Saved Then btnSave.PerformClick()
        Me.DialogResult = DialogResult.OK
        Me.Close()
    End Sub

    Private Sub btnCancel_Click(sender As Object, e As EventArgs) Handles btnCancel.Click
        Me.DialogResult = DialogResult.Cancel
        Me.Close()
    End Sub
End Class