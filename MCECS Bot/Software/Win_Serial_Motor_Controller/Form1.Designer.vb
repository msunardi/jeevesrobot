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
        Me.btnSend = New System.Windows.Forms.Button()
        Me.txtPortNumber = New System.Windows.Forms.TextBox()
        Me.Label4 = New System.Windows.Forms.Label()
        Me.ListBox1 = New System.Windows.Forms.ListBox()
        Me.btnConnect = New System.Windows.Forms.Button()
        Me.btnDisconnect = New System.Windows.Forms.Button()
        Me.txtHeader0 = New System.Windows.Forms.TextBox()
        Me.txtHeader1 = New System.Windows.Forms.TextBox()
        Me.txtOPCode = New System.Windows.Forms.TextBox()
        Me.Label10 = New System.Windows.Forms.Label()
        Me.Label11 = New System.Windows.Forms.Label()
        Me.Label12 = New System.Windows.Forms.Label()
        Me.btnAllStop = New System.Windows.Forms.Button()
        Me.GroupBox1 = New System.Windows.Forms.GroupBox()
        Me.Label9 = New System.Windows.Forms.Label()
        Me.Label8 = New System.Windows.Forms.Label()
        Me.txt4A = New System.Windows.Forms.TextBox()
        Me.txt3A = New System.Windows.Forms.TextBox()
        Me.Label7 = New System.Windows.Forms.Label()
        Me.txt2A = New System.Windows.Forms.TextBox()
        Me.Label6 = New System.Windows.Forms.Label()
        Me.txt1A = New System.Windows.Forms.TextBox()
        Me.txt4B = New System.Windows.Forms.TextBox()
        Me.txt3B = New System.Windows.Forms.TextBox()
        Me.txt2B = New System.Windows.Forms.TextBox()
        Me.txt1B = New System.Windows.Forms.TextBox()
        Me.Label5 = New System.Windows.Forms.Label()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.GroupBox2 = New System.Windows.Forms.GroupBox()
        Me.Label13 = New System.Windows.Forms.Label()
        Me.txtA = New System.Windows.Forms.TextBox()
        Me.txtB = New System.Windows.Forms.TextBox()
        Me.Label14 = New System.Windows.Forms.Label()
        Me.txtCopy = New System.Windows.Forms.Button()
        Me.GroupBox3 = New System.Windows.Forms.GroupBox()
        Me.Label15 = New System.Windows.Forms.Label()
        Me.GroupBox1.SuspendLayout()
        Me.GroupBox2.SuspendLayout()
        Me.GroupBox3.SuspendLayout()
        Me.SuspendLayout()
        '
        'SerialPort1
        '
        '
        'btnSend
        '
        Me.btnSend.Enabled = False
        Me.btnSend.Location = New System.Drawing.Point(144, 319)
        Me.btnSend.Name = "btnSend"
        Me.btnSend.Size = New System.Drawing.Size(78, 27)
        Me.btnSend.TabIndex = 9
        Me.btnSend.Text = "Send"
        Me.btnSend.UseVisualStyleBackColor = True
        '
        'txtPortNumber
        '
        Me.txtPortNumber.Location = New System.Drawing.Point(56, 383)
        Me.txtPortNumber.Name = "txtPortNumber"
        Me.txtPortNumber.Size = New System.Drawing.Size(31, 20)
        Me.txtPortNumber.TabIndex = 10
        Me.txtPortNumber.Text = "4"
        '
        'Label4
        '
        Me.Label4.AutoSize = True
        Me.Label4.Location = New System.Drawing.Point(53, 367)
        Me.Label4.Name = "Label4"
        Me.Label4.Size = New System.Drawing.Size(93, 13)
        Me.Label4.TabIndex = 11
        Me.Label4.Text = "COM Port Number"
        '
        'ListBox1
        '
        Me.ListBox1.FormattingEnabled = True
        Me.ListBox1.Location = New System.Drawing.Point(276, 11)
        Me.ListBox1.Name = "ListBox1"
        Me.ListBox1.Size = New System.Drawing.Size(337, 290)
        Me.ListBox1.TabIndex = 20
        '
        'btnConnect
        '
        Me.btnConnect.Location = New System.Drawing.Point(93, 383)
        Me.btnConnect.Name = "btnConnect"
        Me.btnConnect.Size = New System.Drawing.Size(75, 23)
        Me.btnConnect.TabIndex = 21
        Me.btnConnect.Text = "Connect"
        Me.btnConnect.UseVisualStyleBackColor = True
        '
        'btnDisconnect
        '
        Me.btnDisconnect.Enabled = False
        Me.btnDisconnect.Location = New System.Drawing.Point(174, 383)
        Me.btnDisconnect.Name = "btnDisconnect"
        Me.btnDisconnect.Size = New System.Drawing.Size(75, 23)
        Me.btnDisconnect.TabIndex = 22
        Me.btnDisconnect.Text = "Disconnect"
        Me.btnDisconnect.UseVisualStyleBackColor = True
        '
        'txtHeader0
        '
        Me.txtHeader0.Location = New System.Drawing.Point(79, 12)
        Me.txtHeader0.Name = "txtHeader0"
        Me.txtHeader0.Size = New System.Drawing.Size(46, 20)
        Me.txtHeader0.TabIndex = 23
        Me.txtHeader0.Text = "255"
        '
        'txtHeader1
        '
        Me.txtHeader1.Location = New System.Drawing.Point(79, 38)
        Me.txtHeader1.Name = "txtHeader1"
        Me.txtHeader1.Size = New System.Drawing.Size(46, 20)
        Me.txtHeader1.TabIndex = 24
        Me.txtHeader1.Text = "255"
        '
        'txtOPCode
        '
        Me.txtOPCode.Location = New System.Drawing.Point(201, 26)
        Me.txtOPCode.Name = "txtOPCode"
        Me.txtOPCode.Size = New System.Drawing.Size(46, 20)
        Me.txtOPCode.TabIndex = 25
        Me.txtOPCode.Text = "0"
        '
        'Label10
        '
        Me.Label10.AutoSize = True
        Me.Label10.Location = New System.Drawing.Point(28, 15)
        Me.Label10.Name = "Label10"
        Me.Label10.Size = New System.Drawing.Size(48, 13)
        Me.Label10.TabIndex = 26
        Me.Label10.Text = "Header0"
        '
        'Label11
        '
        Me.Label11.AutoSize = True
        Me.Label11.Location = New System.Drawing.Point(28, 41)
        Me.Label11.Name = "Label11"
        Me.Label11.Size = New System.Drawing.Size(48, 13)
        Me.Label11.TabIndex = 27
        Me.Label11.Text = "Header1"
        '
        'Label12
        '
        Me.Label12.AutoSize = True
        Me.Label12.Location = New System.Drawing.Point(150, 29)
        Me.Label12.Name = "Label12"
        Me.Label12.Size = New System.Drawing.Size(50, 13)
        Me.Label12.TabIndex = 28
        Me.Label12.Text = "OP Code"
        '
        'btnAllStop
        '
        Me.btnAllStop.Enabled = False
        Me.btnAllStop.Location = New System.Drawing.Point(60, 319)
        Me.btnAllStop.Name = "btnAllStop"
        Me.btnAllStop.Size = New System.Drawing.Size(78, 27)
        Me.btnAllStop.TabIndex = 29
        Me.btnAllStop.Text = "All Stop"
        Me.btnAllStop.UseVisualStyleBackColor = True
        '
        'GroupBox1
        '
        Me.GroupBox1.Controls.Add(Me.GroupBox2)
        Me.GroupBox1.Controls.Add(Me.Label9)
        Me.GroupBox1.Controls.Add(Me.Label8)
        Me.GroupBox1.Controls.Add(Me.txt4A)
        Me.GroupBox1.Controls.Add(Me.txt3A)
        Me.GroupBox1.Controls.Add(Me.Label7)
        Me.GroupBox1.Controls.Add(Me.txt2A)
        Me.GroupBox1.Controls.Add(Me.Label6)
        Me.GroupBox1.Controls.Add(Me.txt1A)
        Me.GroupBox1.Controls.Add(Me.txt4B)
        Me.GroupBox1.Controls.Add(Me.txt3B)
        Me.GroupBox1.Controls.Add(Me.txt2B)
        Me.GroupBox1.Controls.Add(Me.txt1B)
        Me.GroupBox1.Controls.Add(Me.Label5)
        Me.GroupBox1.Controls.Add(Me.Label3)
        Me.GroupBox1.Controls.Add(Me.Label2)
        Me.GroupBox1.Controls.Add(Me.Label1)
        Me.GroupBox1.Location = New System.Drawing.Point(12, 77)
        Me.GroupBox1.Name = "GroupBox1"
        Me.GroupBox1.Size = New System.Drawing.Size(249, 198)
        Me.GroupBox1.TabIndex = 30
        Me.GroupBox1.TabStop = False
        '
        'Label9
        '
        Me.Label9.AutoSize = True
        Me.Label9.Location = New System.Drawing.Point(138, 147)
        Me.Label9.Name = "Label9"
        Me.Label9.Size = New System.Drawing.Size(20, 13)
        Me.Label9.TabIndex = 35
        Me.Label9.Text = "4A"
        '
        'Label8
        '
        Me.Label8.AutoSize = True
        Me.Label8.Location = New System.Drawing.Point(13, 147)
        Me.Label8.Name = "Label8"
        Me.Label8.Size = New System.Drawing.Size(20, 13)
        Me.Label8.TabIndex = 34
        Me.Label8.Text = "3A"
        '
        'txt4A
        '
        Me.txt4A.Location = New System.Drawing.Point(141, 163)
        Me.txt4A.Name = "txt4A"
        Me.txt4A.Size = New System.Drawing.Size(46, 20)
        Me.txt4A.TabIndex = 33
        Me.txt4A.Text = "1"
        '
        'txt3A
        '
        Me.txt3A.Location = New System.Drawing.Point(16, 163)
        Me.txt3A.Name = "txt3A"
        Me.txt3A.Size = New System.Drawing.Size(46, 20)
        Me.txt3A.TabIndex = 32
        Me.txt3A.Text = "1"
        '
        'Label7
        '
        Me.Label7.AutoSize = True
        Me.Label7.Location = New System.Drawing.Point(138, 90)
        Me.Label7.Name = "Label7"
        Me.Label7.Size = New System.Drawing.Size(20, 13)
        Me.Label7.TabIndex = 31
        Me.Label7.Text = "2A"
        '
        'txt2A
        '
        Me.txt2A.Location = New System.Drawing.Point(141, 106)
        Me.txt2A.Name = "txt2A"
        Me.txt2A.Size = New System.Drawing.Size(46, 20)
        Me.txt2A.TabIndex = 30
        Me.txt2A.Text = "1"
        '
        'Label6
        '
        Me.Label6.AutoSize = True
        Me.Label6.Location = New System.Drawing.Point(13, 90)
        Me.Label6.Name = "Label6"
        Me.Label6.Size = New System.Drawing.Size(20, 13)
        Me.Label6.TabIndex = 29
        Me.Label6.Text = "1A"
        '
        'txt1A
        '
        Me.txt1A.Location = New System.Drawing.Point(16, 106)
        Me.txt1A.Name = "txt1A"
        Me.txt1A.Size = New System.Drawing.Size(46, 20)
        Me.txt1A.TabIndex = 28
        Me.txt1A.Text = "1"
        '
        'txt4B
        '
        Me.txt4B.Location = New System.Drawing.Point(193, 163)
        Me.txt4B.Name = "txt4B"
        Me.txt4B.Size = New System.Drawing.Size(46, 20)
        Me.txt4B.TabIndex = 27
        Me.txt4B.Text = "10"
        '
        'txt3B
        '
        Me.txt3B.Location = New System.Drawing.Point(68, 163)
        Me.txt3B.Name = "txt3B"
        Me.txt3B.Size = New System.Drawing.Size(46, 20)
        Me.txt3B.TabIndex = 26
        Me.txt3B.Text = "10"
        '
        'txt2B
        '
        Me.txt2B.Location = New System.Drawing.Point(193, 106)
        Me.txt2B.Name = "txt2B"
        Me.txt2B.Size = New System.Drawing.Size(46, 20)
        Me.txt2B.TabIndex = 25
        Me.txt2B.Text = "10"
        '
        'txt1B
        '
        Me.txt1B.Location = New System.Drawing.Point(68, 106)
        Me.txt1B.Name = "txt1B"
        Me.txt1B.Size = New System.Drawing.Size(46, 20)
        Me.txt1B.TabIndex = 24
        Me.txt1B.Text = "10"
        '
        'Label5
        '
        Me.Label5.AutoSize = True
        Me.Label5.Location = New System.Drawing.Point(190, 147)
        Me.Label5.Name = "Label5"
        Me.Label5.Size = New System.Drawing.Size(20, 13)
        Me.Label5.TabIndex = 23
        Me.Label5.Text = "4B"
        '
        'Label3
        '
        Me.Label3.AutoSize = True
        Me.Label3.Location = New System.Drawing.Point(65, 147)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(20, 13)
        Me.Label3.TabIndex = 22
        Me.Label3.Text = "3B"
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Location = New System.Drawing.Point(190, 90)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(20, 13)
        Me.Label2.TabIndex = 21
        Me.Label2.Text = "2B"
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(65, 90)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(20, 13)
        Me.Label1.TabIndex = 20
        Me.Label1.Text = "1B"
        '
        'GroupBox2
        '
        Me.GroupBox2.Controls.Add(Me.txtCopy)
        Me.GroupBox2.Controls.Add(Me.Label13)
        Me.GroupBox2.Controls.Add(Me.txtA)
        Me.GroupBox2.Controls.Add(Me.txtB)
        Me.GroupBox2.Controls.Add(Me.Label14)
        Me.GroupBox2.Location = New System.Drawing.Point(19, 19)
        Me.GroupBox2.Name = "GroupBox2"
        Me.GroupBox2.Size = New System.Drawing.Size(211, 62)
        Me.GroupBox2.TabIndex = 36
        Me.GroupBox2.TabStop = False
        '
        'Label13
        '
        Me.Label13.AutoSize = True
        Me.Label13.Location = New System.Drawing.Point(7, 13)
        Me.Label13.Name = "Label13"
        Me.Label13.Size = New System.Drawing.Size(14, 13)
        Me.Label13.TabIndex = 33
        Me.Label13.Text = "A"
        '
        'txtA
        '
        Me.txtA.Location = New System.Drawing.Point(10, 29)
        Me.txtA.Name = "txtA"
        Me.txtA.Size = New System.Drawing.Size(46, 20)
        Me.txtA.TabIndex = 32
        Me.txtA.Text = "1"
        '
        'txtB
        '
        Me.txtB.Location = New System.Drawing.Point(62, 29)
        Me.txtB.Name = "txtB"
        Me.txtB.Size = New System.Drawing.Size(46, 20)
        Me.txtB.TabIndex = 31
        Me.txtB.Text = "10"
        '
        'Label14
        '
        Me.Label14.AutoSize = True
        Me.Label14.Location = New System.Drawing.Point(59, 13)
        Me.Label14.Name = "Label14"
        Me.Label14.Size = New System.Drawing.Size(14, 13)
        Me.Label14.TabIndex = 30
        Me.Label14.Text = "B"
        '
        'txtCopy
        '
        Me.txtCopy.Location = New System.Drawing.Point(133, 23)
        Me.txtCopy.Name = "txtCopy"
        Me.txtCopy.Size = New System.Drawing.Size(59, 25)
        Me.txtCopy.TabIndex = 34
        Me.txtCopy.Text = "Copy"
        Me.txtCopy.UseVisualStyleBackColor = True
        '
        'GroupBox3
        '
        Me.GroupBox3.Controls.Add(Me.Label15)
        Me.GroupBox3.Location = New System.Drawing.Point(302, 319)
        Me.GroupBox3.Name = "GroupBox3"
        Me.GroupBox3.Size = New System.Drawing.Size(288, 112)
        Me.GroupBox3.TabIndex = 31
        Me.GroupBox3.TabStop = False
        '
        'Label15
        '
        Me.Label15.AutoSize = True
        Me.Label15.Location = New System.Drawing.Point(6, 16)
        Me.Label15.Name = "Label15"
        Me.Label15.Size = New System.Drawing.Size(153, 91)
        Me.Label15.TabIndex = 32
        Me.Label15.Text = "OP Code 00:" & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "A = Direction (0 - Rev, 1 - Fwd)" & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "B = Speed (0-127)" & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "OP Code 01:" & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & _
    "A = Speed (cm/s)" & Global.Microsoft.VisualBasic.ChrW(13) & Global.Microsoft.VisualBasic.ChrW(10) & "B = Distance (m)"
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(630, 443)
        Me.Controls.Add(Me.GroupBox3)
        Me.Controls.Add(Me.GroupBox1)
        Me.Controls.Add(Me.btnAllStop)
        Me.Controls.Add(Me.Label12)
        Me.Controls.Add(Me.Label11)
        Me.Controls.Add(Me.Label10)
        Me.Controls.Add(Me.txtOPCode)
        Me.Controls.Add(Me.txtHeader1)
        Me.Controls.Add(Me.txtHeader0)
        Me.Controls.Add(Me.btnDisconnect)
        Me.Controls.Add(Me.btnConnect)
        Me.Controls.Add(Me.ListBox1)
        Me.Controls.Add(Me.Label4)
        Me.Controls.Add(Me.txtPortNumber)
        Me.Controls.Add(Me.btnSend)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.GroupBox1.ResumeLayout(False)
        Me.GroupBox1.PerformLayout()
        Me.GroupBox2.ResumeLayout(False)
        Me.GroupBox2.PerformLayout()
        Me.GroupBox3.ResumeLayout(False)
        Me.GroupBox3.PerformLayout()
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents SerialPort1 As System.IO.Ports.SerialPort
    Friend WithEvents btnSend As System.Windows.Forms.Button
    Friend WithEvents txtPortNumber As System.Windows.Forms.TextBox
    Friend WithEvents Label4 As System.Windows.Forms.Label
    Friend WithEvents ListBox1 As System.Windows.Forms.ListBox
    Friend WithEvents btnConnect As System.Windows.Forms.Button
    Friend WithEvents btnDisconnect As System.Windows.Forms.Button
    Friend WithEvents txtHeader0 As System.Windows.Forms.TextBox
    Friend WithEvents txtHeader1 As System.Windows.Forms.TextBox
    Friend WithEvents txtOPCode As System.Windows.Forms.TextBox
    Friend WithEvents Label10 As System.Windows.Forms.Label
    Friend WithEvents Label11 As System.Windows.Forms.Label
    Friend WithEvents Label12 As System.Windows.Forms.Label
    Friend WithEvents btnAllStop As System.Windows.Forms.Button
    Friend WithEvents GroupBox1 As System.Windows.Forms.GroupBox
    Friend WithEvents Label9 As System.Windows.Forms.Label
    Friend WithEvents Label8 As System.Windows.Forms.Label
    Friend WithEvents txt4A As System.Windows.Forms.TextBox
    Friend WithEvents txt3A As System.Windows.Forms.TextBox
    Friend WithEvents Label7 As System.Windows.Forms.Label
    Friend WithEvents txt2A As System.Windows.Forms.TextBox
    Friend WithEvents Label6 As System.Windows.Forms.Label
    Friend WithEvents txt1A As System.Windows.Forms.TextBox
    Friend WithEvents txt4B As System.Windows.Forms.TextBox
    Friend WithEvents txt3B As System.Windows.Forms.TextBox
    Friend WithEvents txt2B As System.Windows.Forms.TextBox
    Friend WithEvents txt1B As System.Windows.Forms.TextBox
    Friend WithEvents Label5 As System.Windows.Forms.Label
    Friend WithEvents Label3 As System.Windows.Forms.Label
    Friend WithEvents Label2 As System.Windows.Forms.Label
    Friend WithEvents Label1 As System.Windows.Forms.Label
    Friend WithEvents GroupBox2 As System.Windows.Forms.GroupBox
    Friend WithEvents txtCopy As System.Windows.Forms.Button
    Friend WithEvents Label13 As System.Windows.Forms.Label
    Friend WithEvents txtA As System.Windows.Forms.TextBox
    Friend WithEvents txtB As System.Windows.Forms.TextBox
    Friend WithEvents Label14 As System.Windows.Forms.Label
    Friend WithEvents GroupBox3 As System.Windows.Forms.GroupBox
    Friend WithEvents Label15 As System.Windows.Forms.Label

End Class
