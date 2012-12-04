Public Class Form1

    Private Sub btnSend_Click(sender As Object, e As EventArgs) Handles btnSend.Click
        Dim byteFrame() As Byte
        Dim charFrame() As Char
        Dim byteDirection As Byte

        ReDim byteFrame(2)
        byteFrame(0) = Convert.ToByte(CInt(txtHeader0.Text))
        byteFrame(1) = Convert.ToByte(CInt(txtHeader1.Text))
        byteFrame(2) = Convert.ToByte(CInt(txtOPCode.Text))

        Select Case byteFrame(2)
            Case 0
                ReDim Preserve byteFrame(7)
                byteDirection = Convert.ToByte(txt1A.Text) + Convert.ToByte(CInt(txt2A.Text) * 2) + Convert.ToByte(CInt(txt3A.Text) * 4) + Convert.ToByte(CInt(txt4A.Text) * 8)
                byteFrame(3) = Convert.ToByte(byteDirection)
                byteFrame(4) = Convert.ToByte(CInt(txt1B.Text))
                byteFrame(5) = Convert.ToByte(CInt(txt2B.Text))
                byteFrame(6) = Convert.ToByte(CInt(txt3B.Text))
                byteFrame(7) = Convert.ToByte(CInt(txt4B.Text))

                Try
                    SerialPort1.Write(byteFrame, 0, byteFrame.Length)
                Catch ex As Exception
                    MsgBox("Error writting to port")
                End Try

            Case 1
                ReDim Preserve byteFrame(6)
                byteFrame(3) = Convert.ToByte(CInt(txt1B.Text))
                byteFrame(4) = Convert.ToByte(CInt(txt2B.Text))
                byteFrame(5) = Convert.ToByte(CInt(txt3B.Text))
                byteFrame(6) = Convert.ToByte(CInt(txt4B.Text))
                ReDim charFrame(3)
                charFrame(0) = Convert.ToChar(CInt(txt1A.Text))
                charFrame(1) = Convert.ToChar(CInt(txt2A.Text))
                charFrame(2) = Convert.ToChar(CInt(txt3A.Text))
                charFrame(3) = Convert.ToChar(CInt(txt4A.Text))

                Try
                    SerialPort1.Write(byteFrame, 0, byteFrame.Length)
                    SerialPort1.Write(charFrame, 0, charFrame.Length)
                Catch ex As Exception
                    MsgBox("Error writting to port")
                End Try

        End Select

    End Sub

    Private Sub SerialPort1_DataReceived(sender As Object, e As IO.Ports.SerialDataReceivedEventArgs) Handles SerialPort1.DataReceived
        Try
            IncomingData(SerialPort1.ReadLine)

        Catch ex As Exception

        End Try

    End Sub

    Private Sub btnConnect_Click(sender As Object, e As EventArgs) Handles btnConnect.Click
        SerialPort1.PortName = "COM" & CInt(txtPortNumber.Text)
        SerialPort1.BaudRate = 9600
        SerialPort1.DataBits = 8
        SerialPort1.Parity = IO.Ports.Parity.None
        SerialPort1.StopBits = IO.Ports.StopBits.One
        SerialPort1.Handshake = IO.Ports.Handshake.None

        Try
            SerialPort1.Open()

        Catch ex As Exception
            MsgBox("Error opening serial port")
            Exit Sub

        End Try

        btnConnect.Enabled = False
        btnDisconnect.Enabled = True
        btnSend.Enabled = True
        btnAllStop.Enabled = True

    End Sub

    Private Sub btnDisconnect_Click(sender As Object, e As EventArgs) Handles btnDisconnect.Click
        Try
            SerialPort1.Close()

        Catch ex As Exception

        End Try

        btnDisconnect.Enabled = False
        btnConnect.Enabled = True
        btnSend.Enabled = False
        btnAllStop.Enabled = False

    End Sub

    Private Delegate Sub IncomingDataDelegate(ByVal sData As String)

    Public Sub IncomingData(ByVal sData As String)
        If ListBox1.InvokeRequired Then
            Dim del As New IncomingDataDelegate(AddressOf IncomingData)
            ListBox1.Invoke(del, New Object() {sData})
        Else
            ListBox1.Items.Add(sData)
            ListBox1.SelectedIndex = ListBox1.Items.Count - 1
            'Debug.Print(ListBox1.Items.Count)

        End If


    End Sub

    Private Sub Form1_FormClosing(sender As Object, e As FormClosingEventArgs) Handles Me.FormClosing
        btnDisconnect_Click(Nothing, Nothing)

    End Sub

    Private Sub btnAllStop_Click(sender As Object, e As EventArgs) Handles btnAllStop.Click
        txtOPCode.Text = 0
        txt1B.Text = 0
        txt2B.Text = 0
        txt3B.Text = 0
        txt4B.Text = 0
        btnSend_Click(Nothing, Nothing)

    End Sub

    Private Sub txtCopy_Click(sender As Object, e As EventArgs) Handles txtCopy.Click
        txt1A.Text = txtA.Text
        txt2A.Text = txtA.Text
        txt3A.Text = txtA.Text
        txt4A.Text = txtA.Text

        txt1B.Text = txtB.Text
        txt2B.Text = txtB.Text
        txt3B.Text = txtB.Text
        txt4B.Text = txtB.Text

    End Sub
End Class
