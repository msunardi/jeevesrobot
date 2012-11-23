Public Class Form1

    Private Sub btnSend_Click(sender As Object, e As EventArgs) Handles btnSend.Click
        Dim serFrame(12) As Byte

        serFrame(0) = Convert.ToByte(100)
        serFrame(1) = Convert.ToByte(128)
        serFrame(2) = Convert.ToByte(0)
        serFrame(3) = Convert.ToByte(0)
        serFrame(4) = Convert.ToByte(CInt(txt1Dir.Text))
        serFrame(5) = Convert.ToByte(CInt(txt1Speed.Text))
        serFrame(6) = Convert.ToByte(CInt(txt2Dir.Text))
        serFrame(7) = Convert.ToByte(CInt(txt2Speed.Text))
        serFrame(8) = Convert.ToByte(CInt(txt3Dir.Text))
        serFrame(9) = Convert.ToByte(CInt(txt3Speed.Text))
        serFrame(10) = Convert.ToByte(CInt(txt4Dir.Text))
        serFrame(11) = Convert.ToByte(CInt(txt4Speed.Text))
        serFrame(12) = Convert.ToByte(0)

        SerialPort1.Write(serFrame, 0, 12)

    End Sub

    Private Sub SerialPort1_DataReceived(sender As Object, e As IO.Ports.SerialDataReceivedEventArgs) Handles SerialPort1.DataReceived
        IncomingData(SerialPort1.ReadLine)

    End Sub

    Private Sub btnConnect_Click(sender As Object, e As EventArgs) Handles btnConnect.Click
        Try
            SerialPort1.PortName = "COM" & CInt(txtPortNumber.Text)
            SerialPort1.BaudRate = 9600
            SerialPort1.DataBits = 8
            SerialPort1.Parity = IO.Ports.Parity.None
            SerialPort1.StopBits = IO.Ports.StopBits.One
            SerialPort1.Handshake = IO.Ports.Handshake.None
            SerialPort1.Open()

        Catch ex As Exception
            MsgBox("Error opening serial port")
            Exit Sub

        End Try

        btnConnect.Enabled = False
        btnDisconnect.Enabled = True

    End Sub

    Private Sub btnDisconnect_Click(sender As Object, e As EventArgs) Handles btnDisconnect.Click
        SerialPort1.Close()

        btnDisconnect.Enabled = False
        btnConnect.Enabled = True

    End Sub

    Private Delegate Sub IncomingDataDelegate(ByVal sData As String)
    'Dim del As New IncomingDataDelegate(AddressOf IncomingData)

    Public Sub IncomingData(ByVal sData As String)
        If ListBox1.InvokeRequired Then
            Dim del As New IncomingDataDelegate(AddressOf IncomingData)
            ListBox1.Invoke(del, New Object() {sData})
        Else
            ListBox1.Items.Add(sData)

        End If


    End Sub

    Private Sub Form1_FormClosing(sender As Object, e As FormClosingEventArgs) Handles Me.FormClosing
        btnDisconnect_Click(Nothing, Nothing)

    End Sub
End Class
