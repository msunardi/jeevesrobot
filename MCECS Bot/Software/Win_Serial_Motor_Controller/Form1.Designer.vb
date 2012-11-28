<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class Form1
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Me.components = New System.ComponentModel.Container()
        Me.SerialPort1 = New System.IO.Ports.SerialPort(Me.components)
        Me.Label1 = New System.Windows.Forms.Label()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.Label5 = New System.Windows.Forms.Label()
        Me.txt1Speed = New System.Windows.Forms.TextBox()
        Me.txt2Speed = New System.Windows.Forms.TextBox()
        Me.txt3Speed = New System.Windows.Forms.TextBox()
        Me.txt4Speed = New System.Windows.Forms.TextBox()
        Me.btnSend = New System.Windows.Forms.Button()
        Me.txtPortNumber = New System.Windows.Forms.TextBox()
        Me.Label4 = New System.Windows.Forms.Label()
        Me.txt1Dir = New System.Windows.Forms.TextBox()
        Me.Label6 = New System.Windows.Forms.Label()
        Me.txt2Dir = New System.Windows.Forms.TextBox()
        Me.Label7 = New System.Windows.Forms.Label()
        Me.txt3Dir = New System.Windows.Forms.TextBox()
        Me.txt4Dir = New System.Windows.Forms.TextBox()
        Me.Label8 = New System.Windows.Forms.Label()
        Me.Label9 = New System.Windows.Forms.Label()
        Me.ListBox1 = New System.Windows.Forms.ListBox()
        Me.btnConnect = New System.Windows.Forms.Button()
        Me.btnDisconnect = New System.Windows.Forms.Button()
        Me.txtHeader0 = New System.Windows.Forms.TextBox()
        Me.txtHeader1 = New System.Windows.Forms.TextBox()
        Me.txtOPCode = New System.Windows.Forms.TextBox()
        Me.Label10 = New System.Windows.Forms.Label()
        Me.Label11 = New System.Windows.Forms.Label()
        Me.Label12 = New System.Windows.Forms.Label()
        Me.SuspendLayout()
        '
        'SerialPort1
        '
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(75, 104)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(49, 13)
        Me.Label1.TabIndex = 0
        Me.Label1.Text = "1 (0-127)"
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Location = New System.Drawing.Point(200, 104)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(49, 13)
        Me.Label2.TabIndex = 1
        Me.Label2.Text = "2 (0-127)"
        '
        'Label3
        '
        Me.Label3.AutoSize = True
        Me.Label3.Location = New System.Drawing.Point(75, 161)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(49, 13)
        Me.Label3.TabIndex = 2
        Me.Label3.Text = "3 (0-127)"
        '
        'Label5
        '
        Me.Label5.AutoSize = True
        Me.Label5.Location = New System.Drawing.Point(200, 161)
        Me.Label5.Name = "Label5"
        Me.Label5.Size = New System.Drawing.Size(49, 13)
        Me.Label5.TabIndex = 4
        Me.Label5.Text = "4 (0-127)"
        '
        'txt1Speed
        '
        Me.txt1Speed.Location = New System.Drawing.Point(78, 120)
        Me.txt1Speed.Name = "txt1Speed"
        Me.txt1Speed.Size = New System.Drawing.Size(46, 20)
        Me.txt1Speed.TabIndex = 5
        Me.txt1Speed.Text = "10"
        '
        'txt2Speed
        '
        Me.txt2Speed.Location = New System.Drawing.Point(203, 120)
        Me.txt2Speed.Name = "txt2Speed"
        Me.txt2Speed.Size = New System.Drawing.Size(46, 20)
        Me.txt2Speed.TabIndex = 6
        Me.txt2Speed.Text = "15"
        '
        'txt3Speed
        '
        Me.txt3Speed.Location = New System.Drawing.Point(78, 177)
        Me.txt3Speed.Name = "txt3Speed"
        Me.txt3Speed.Size = New System.Drawing.Size(46, 20)
        Me.txt3Speed.TabIndex = 7
        Me.txt3Speed.Text = "100"
        '
        'txt4Speed
        '
        Me.txt4Speed.Location = New System.Drawing.Point(203, 177)
        Me.txt4Speed.Name = "txt4Speed"
        Me.txt4Speed.Size = New System.Drawing.Size(46, 20)
        Me.txt4Speed.TabIndex = 8
        Me.txt4Speed.Text = "58"
        '
        'btnSend
        '
        Me.btnSend.Location = New System.Drawing.Point(171, 203)
        Me.btnSend.Name = "btnSend"
        Me.btnSend.Size = New System.Drawing.Size(78, 27)
        Me.btnSend.TabIndex = 9
        Me.btnSend.Text = "Send"
        Me.btnSend.UseVisualStyleBackColor = True
        '
        'txtPortNumber
        '
        Me.txtPortNumber.Location = New System.Drawing.Point(45, 254)
        Me.txtPortNumber.Name = "txtPortNumber"
        Me.txtPortNumber.Size = New System.Drawing.Size(31, 20)
        Me.txtPortNumber.TabIndex = 10
        Me.txtPortNumber.Text = "4"
        '
        'Label4
        '
        Me.Label4.AutoSize = True
        Me.Label4.Location = New System.Drawing.Point(42, 238)
        Me.Label4.Name = "Label4"
        Me.Label4.Size = New System.Drawing.Size(93, 13)
        Me.Label4.TabIndex = 11
        Me.Label4.Text = "COM Port Number"
        '
        'txt1Dir
        '
        Me.txt1Dir.Location = New System.Drawing.Point(26, 120)
        Me.txt1Dir.Name = "txt1Dir"
        Me.txt1Dir.Size = New System.Drawing.Size(46, 20)
        Me.txt1Dir.TabIndex = 12
        Me.txt1Dir.Text = "1"
        '
        'Label6
        '
        Me.Label6.AutoSize = True
        Me.Label6.Location = New System.Drawing.Point(23, 104)
        Me.Label6.Name = "Label6"
        Me.Label6.Size = New System.Drawing.Size(53, 13)
        Me.Label6.TabIndex = 13
        Me.Label6.Text = "1 Dir (0-1)"
        '
        'txt2Dir
        '
        Me.txt2Dir.Location = New System.Drawing.Point(151, 120)
        Me.txt2Dir.Name = "txt2Dir"
        Me.txt2Dir.Size = New System.Drawing.Size(46, 20)
        Me.txt2Dir.TabIndex = 14
        Me.txt2Dir.Text = "1"
        '
        'Label7
        '
        Me.Label7.AutoSize = True
        Me.Label7.Location = New System.Drawing.Point(148, 104)
        Me.Label7.Name = "Label7"
        Me.Label7.Size = New System.Drawing.Size(53, 13)
        Me.Label7.TabIndex = 15
        Me.Label7.Text = "2 Dir (0-1)"
        '
        'txt3Dir
        '
        Me.txt3Dir.Location = New System.Drawing.Point(26, 177)
        Me.txt3Dir.Name = "txt3Dir"
        Me.txt3Dir.Size = New System.Drawing.Size(46, 20)
        Me.txt3Dir.TabIndex = 16
        Me.txt3Dir.Text = "0"
        '
        'txt4Dir
        '
        Me.txt4Dir.Location = New System.Drawing.Point(151, 177)
        Me.txt4Dir.Name = "txt4Dir"
        Me.txt4Dir.Size = New System.Drawing.Size(46, 20)
        Me.txt4Dir.TabIndex = 17
        Me.txt4Dir.Text = "1"
        '
        'Label8
        '
        Me.Label8.AutoSize = True
        Me.Label8.Location = New System.Drawing.Point(23, 161)
        Me.Label8.Name = "Label8"
        Me.Label8.Size = New System.Drawing.Size(53, 13)
        Me.Label8.TabIndex = 18
        Me.Label8.Text = "3 Dir (0-1)"
        '
        'Label9
        '
        Me.Label9.AutoSize = True
        Me.Label9.Location = New System.Drawing.Point(148, 161)
        Me.Label9.Name = "Label9"
        Me.Label9.Size = New System.Drawing.Size(53, 13)
        Me.Label9.TabIndex = 19
        Me.Label9.Text = "4 Dir (0-1)"
        '
        'ListBox1
        '
        Me.ListBox1.FormattingEnabled = True
        Me.ListBox1.Location = New System.Drawing.Point(276, 11)
        Me.ListBox1.Name = "ListBox1"
        Me.ListBox1.Size = New System.Drawing.Size(337, 264)
        Me.ListBox1.TabIndex = 20
        '
        'btnConnect
        '
        Me.btnConnect.Location = New System.Drawing.Point(82, 254)
        Me.btnConnect.Name = "btnConnect"
        Me.btnConnect.Size = New System.Drawing.Size(75, 23)
        Me.btnConnect.TabIndex = 21
        Me.btnConnect.Text = "Connect"
        Me.btnConnect.UseVisualStyleBackColor = True
        '
        'btnDisconnect
        '
        Me.btnDisconnect.Enabled = False
        Me.btnDisconnect.Location = New System.Drawing.Point(163, 254)
        Me.btnDisconnect.Name = "btnDisconnect"
        Me.btnDisconnect.Size = New System.Drawing.Size(75, 23)
        Me.btnDisconnect.TabIndex = 22
        Me.btnDisconnect.Text = "Disconnect"
        Me.btnDisconnect.UseVisualStyleBackColor = True
        '
        'txtHeader0
        '
        Me.txtHeader0.Location = New System.Drawing.Point(126, 13)
        Me.txtHeader0.Name = "txtHeader0"
        Me.txtHeader0.Size = New System.Drawing.Size(46, 20)
        Me.txtHeader0.TabIndex = 23
        Me.txtHeader0.Text = "255"
        '
        'txtHeader1
        '
        Me.txtHeader1.Location = New System.Drawing.Point(126, 39)
        Me.txtHeader1.Name = "txtHeader1"
        Me.txtHeader1.Size = New System.Drawing.Size(46, 20)
        Me.txtHeader1.TabIndex = 24
        Me.txtHeader1.Text = "255"
        '
        'txtOPCode
        '
        Me.txtOPCode.Location = New System.Drawing.Point(126, 65)
        Me.txtOPCode.Name = "txtOPCode"
        Me.txtOPCode.Size = New System.Drawing.Size(46, 20)
        Me.txtOPCode.TabIndex = 25
        Me.txtOPCode.Text = "0"
        '
        'Label10
        '
        Me.Label10.AutoSize = True
        Me.Label10.Location = New System.Drawing.Point(75, 16)
        Me.Label10.Name = "Label10"
        Me.Label10.Size = New System.Drawing.Size(48, 13)
        Me.Label10.TabIndex = 26
        Me.Label10.Text = "Header0"
        '
        'Label11
        '
        Me.Label11.AutoSize = True
        Me.Label11.Location = New System.Drawing.Point(75, 42)
        Me.Label11.Name = "Label11"
        Me.Label11.Size = New System.Drawing.Size(48, 13)
        Me.Label11.TabIndex = 27
        Me.Label11.Text = "Header1"
        '
        'Label12
        '
        Me.Label12.AutoSize = True
        Me.Label12.Location = New System.Drawing.Point(75, 68)
        Me.Label12.Name = "Label12"
        Me.Label12.Size = New System.Drawing.Size(50, 13)
        Me.Label12.TabIndex = 28
        Me.Label12.Text = "OP Code"
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(625, 292)
        Me.Controls.Add(Me.Label12)
        Me.Controls.Add(Me.Label11)
        Me.Controls.Add(Me.Label10)
        Me.Controls.Add(Me.txtOPCode)
        Me.Controls.Add(Me.txtHeader1)
        Me.Controls.Add(Me.txtHeader0)
        Me.Controls.Add(Me.btnDisconnect)
        Me.Controls.Add(Me.btnConnect)
        Me.Controls.Add(Me.ListBox1)
        Me.Controls.Add(Me.Label9)
        Me.Controls.Add(Me.Label8)
        Me.Controls.Add(Me.txt4Dir)
        Me.Controls.Add(Me.txt3Dir)
        Me.Controls.Add(Me.Label7)
        Me.Controls.Add(Me.txt2Dir)
        Me.Controls.Add(Me.Label6)
        Me.Controls.Add(Me.txt1Dir)
        Me.Controls.Add(Me.Label4)
        Me.Controls.Add(Me.txtPortNumber)
        Me.Controls.Add(Me.btnSend)
        Me.Controls.Add(Me.txt4Speed)
        Me.Controls.Add(Me.txt3Speed)
        Me.Controls.Add(Me.txt2Speed)
        Me.Controls.Add(Me.txt1Speed)
        Me.Controls.Add(Me.Label5)
        Me.Controls.Add(Me.Label3)
        Me.Controls.Add(Me.Label2)
        Me.Controls.Add(Me.Label1)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents SerialPort1 As System.IO.Ports.SerialPort
    Friend WithEvents Label1 As System.Windows.Forms.Label
    Friend WithEvents Label2 As System.Windows.Forms.Label
    Friend WithEvents Label3 As System.Windows.Forms.Label
    Friend WithEvents Label5 As System.Windows.Forms.Label
    Friend WithEvents txt1Speed As System.Windows.Forms.TextBox
    Friend WithEvents txt2Speed As System.Windows.Forms.TextBox
    Friend WithEvents txt3Speed As System.Windows.Forms.TextBox
    Friend WithEvents txt4Speed As System.Windows.Forms.TextBox
    Friend WithEvents btnSend As System.Windows.Forms.Button
    Friend WithEvents txtPortNumber As System.Windows.Forms.TextBox
    Friend WithEvents Label4 As System.Windows.Forms.Label
    Friend WithEvents txt1Dir As System.Windows.Forms.TextBox
    Friend WithEvents Label6 As System.Windows.Forms.Label
    Friend WithEvents txt2Dir As System.Windows.Forms.TextBox
    Friend WithEvents Label7 As System.Windows.Forms.Label
    Friend WithEvents txt3Dir As System.Windows.Forms.TextBox
    Friend WithEvents txt4Dir As System.Windows.Forms.TextBox
    Friend WithEvents Label8 As System.Windows.Forms.Label
    Friend WithEvents Label9 As System.Windows.Forms.Label
    Friend WithEvents ListBox1 As System.Windows.Forms.ListBox
    Friend WithEvents btnConnect As System.Windows.Forms.Button
    Friend WithEvents btnDisconnect As System.Windows.Forms.Button
    Friend WithEvents txtHeader0 As System.Windows.Forms.TextBox
    Friend WithEvents txtHeader1 As System.Windows.Forms.TextBox
    Friend WithEvents txtOPCode As System.Windows.Forms.TextBox
    Friend WithEvents Label10 As System.Windows.Forms.Label
    Friend WithEvents Label11 As System.Windows.Forms.Label
    Friend WithEvents Label12 As System.Windows.Forms.Label

End Class
