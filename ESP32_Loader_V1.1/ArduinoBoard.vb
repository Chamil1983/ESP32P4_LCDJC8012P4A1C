Public Class ArduinoBoard
    Public Property Name As String
    Public Property FQBN As String
    Public Property ConfigOptions As Dictionary(Of String, List(Of String))

    Public Shared Function ParseBoardsTxt(path As String) As List(Of ArduinoBoard)
        Dim boards As New List(Of ArduinoBoard)
        Dim lines = IO.File.ReadAllLines(path)
        Dim boardDict As New Dictionary(Of String, ArduinoBoard)

        For Each line In lines
            If String.IsNullOrWhiteSpace(line) OrElse line.TrimStart().StartsWith("#") Then
                Continue For  ' Skip comments and empty lines
            End If

            If line.Contains(".name=") Then
                Dim id = line.Substring(0, line.IndexOf(".name="))
                Dim name = line.Substring(line.IndexOf("=") + 1)

                ' Use the correct format for the FQBN - don't hardcode esp32:esp32:
                ' For ESP32, the correct format is typically just esp32:esp32:boardtype
                boardDict(id) = New ArduinoBoard With {
                    .Name = name,
                    .FQBN = $"esp32:esp32:{id}",
                    .ConfigOptions = New Dictionary(Of String, List(Of String))()
                }
            End If
        Next

        ' Parse configs
        For Each line In lines
            If String.IsNullOrWhiteSpace(line) OrElse line.TrimStart().StartsWith("#") Then
                Continue For  ' Skip comments and empty lines
            End If

            If line.Contains(".menu.") Then
                Try
                    Dim parts = line.Split({"."c}, StringSplitOptions.None)
                    If parts.Length >= 4 Then
                        Dim id = parts(0)
                        Dim optionType = parts(2) ' e.g. FlashFreq
                        Dim optionValue = parts(3).Split("="c)(0) ' e.g. 80
                        Dim displayValue = line.Substring(line.IndexOf("=") + 1)

                        If boardDict.ContainsKey(id) Then
                            If Not boardDict(id).ConfigOptions.ContainsKey(optionType) Then
                                boardDict(id).ConfigOptions(optionType) = New List(Of String)()
                            End If

                            ' For the board config, we need the option value and display name
                            Dim optionPair = $"{optionValue}={displayValue}"
                            If Not boardDict(id).ConfigOptions(optionType).Contains(optionPair) Then
                                boardDict(id).ConfigOptions(optionType).Add(optionPair)
                            End If
                        End If
                    End If
                Catch ex As Exception
                    ' Just skip problematic entries
                    Continue For
                End Try
            End If
        Next

        Return boardDict.Values.ToList()
    End Function
End Class

Public Class ArduinoBoardConfig
    Public Property Options As New Dictionary(Of String, String)

    ''' <summary>
    ''' Formats board options in the correct arduino-cli format
    ''' </summary>
    Public Function ToCliOptions() As String
        Dim result As String = ""
        For Each kvp In Options
            ' Format is option=value (without colon prefix)
            result &= ":" & kvp.Key & "=" & kvp.Value
        Next
        Return result
    End Function
End Class