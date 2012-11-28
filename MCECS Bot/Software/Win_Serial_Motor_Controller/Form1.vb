Public Class Form1

    Private Sub btnSend_Click(sender As Object, e As EventArgs) Handles btnSend.Click
        Dim byteFrame(7) As Byte
        Dim byteDirection As Byte

        byteDirection = Convert.ToByte(txt1Dir.Text) + Convert.ToByte(CInt(txt2Dir.Text) * 2) + Convert.ToByte(CInt(txt3Dir.Text) * 4) + Convert.ToByte(CInt(txt4Dir.Text) * 8)


        byteFrame(0) = Convert.ToByte(CInt(txtHeader0.Text))
        byteFrame(1) = Convert.ToByte(CInt(txtHeader1.Text))
        byteFrame(2) = Convert.ToByte(CInt(txtOPCode.Text))
        byteFrame(3) = Convert.ToByte(byteDirection)
        byteFrame(4) = Convert.ToByte(CInt(txt1Speed.Text))
        byteFrame(5) = Convert.ToByte(CInt(txt2Speed.Text))
        byteFrame(6) = Convert.ToByte(CInt(txt3Speed.Text))
        byteFrame(7) = Convert.ToByte(CInt(txt4Speed.Text))

        Try
            SerialPort1.Write(byteFrame, 0, 8)
        Catch ex As Exception
            MsgBox("Error writting to port")
        End Try


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
End Class
