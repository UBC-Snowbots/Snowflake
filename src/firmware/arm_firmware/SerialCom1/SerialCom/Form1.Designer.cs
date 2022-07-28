
namespace SerialCom
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.webBrowser1 = new System.Windows.Forms.WebBrowser();
            this.btnCLOSE = new System.Windows.Forms.Button();
            this.btnSENDDATA = new System.Windows.Forms.Button();
            this.cBoxPARITYBITS = new System.Windows.Forms.ComboBox();
            this.cBoxSTOPBITS = new System.Windows.Forms.ComboBox();
            this.cBoxDATABITS = new System.Windows.Forms.ComboBox();
            this.cBoxBAUDRATE = new System.Windows.Forms.ComboBox();
            this.cBoxCOMPORT = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.tBoxDATAOUT = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.btnMOVE = new System.Windows.Forms.Button();
            this.btnHOME = new System.Windows.Forms.Button();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.label18 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label20 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label21 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label22 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.btnOPEN = new System.Windows.Forms.Button();
            this.label5 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.btnINC = new System.Windows.Forms.RadioButton();
            this.btnABS = new System.Windows.Forms.RadioButton();
            this.groubox5 = new System.Windows.Forms.GroupBox();
            this.a1prog = new System.Windows.Forms.ProgressBar();
            this.a2prog = new System.Windows.Forms.ProgressBar();
            this.a5prog = new System.Windows.Forms.ProgressBar();
            this.a4prog = new System.Windows.Forms.ProgressBar();
            this.a3prog = new System.Windows.Forms.ProgressBar();
            this.a6prog = new System.Windows.Forms.ProgressBar();
            this.groupBox1.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groubox5.SuspendLayout();
            this.SuspendLayout();
            // 
            // webBrowser1
            // 
            this.webBrowser1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.webBrowser1.Location = new System.Drawing.Point(0, 0);
            this.webBrowser1.MinimumSize = new System.Drawing.Size(20, 20);
            this.webBrowser1.Name = "webBrowser1";
            this.webBrowser1.Size = new System.Drawing.Size(1050, 712);
            this.webBrowser1.TabIndex = 0;
            this.webBrowser1.DocumentCompleted += new System.Windows.Forms.WebBrowserDocumentCompletedEventHandler(this.webBrowser1_DocumentCompleted);
            // 
            // btnCLOSE
            // 
            this.btnCLOSE.Location = new System.Drawing.Point(146, 25);
            this.btnCLOSE.Name = "btnCLOSE";
            this.btnCLOSE.Size = new System.Drawing.Size(107, 47);
            this.btnCLOSE.TabIndex = 2;
            this.btnCLOSE.Text = "CLOSE";
            this.btnCLOSE.UseVisualStyleBackColor = true;
            this.btnCLOSE.Click += new System.EventHandler(this.button2_Click);
            // 
            // btnSENDDATA
            // 
            this.btnSENDDATA.Location = new System.Drawing.Point(310, 263);
            this.btnSENDDATA.Name = "btnSENDDATA";
            this.btnSENDDATA.Size = new System.Drawing.Size(86, 102);
            this.btnSENDDATA.TabIndex = 3;
            this.btnSENDDATA.Text = "SEND DATA";
            this.btnSENDDATA.UseVisualStyleBackColor = true;
            this.btnSENDDATA.Click += new System.EventHandler(this.button3_Click);
            // 
            // cBoxPARITYBITS
            // 
            this.cBoxPARITYBITS.FormattingEnabled = true;
            this.cBoxPARITYBITS.Items.AddRange(new object[] {
            "None",
            "Odd",
            "Even"});
            this.cBoxPARITYBITS.Location = new System.Drawing.Point(133, 183);
            this.cBoxPARITYBITS.Name = "cBoxPARITYBITS";
            this.cBoxPARITYBITS.Size = new System.Drawing.Size(272, 28);
            this.cBoxPARITYBITS.TabIndex = 4;
            this.cBoxPARITYBITS.Text = "None";
            // 
            // cBoxSTOPBITS
            // 
            this.cBoxSTOPBITS.FormattingEnabled = true;
            this.cBoxSTOPBITS.Items.AddRange(new object[] {
            "One",
            "Two"});
            this.cBoxSTOPBITS.Location = new System.Drawing.Point(133, 147);
            this.cBoxSTOPBITS.Name = "cBoxSTOPBITS";
            this.cBoxSTOPBITS.Size = new System.Drawing.Size(272, 28);
            this.cBoxSTOPBITS.TabIndex = 5;
            this.cBoxSTOPBITS.Text = "One";
            // 
            // cBoxDATABITS
            // 
            this.cBoxDATABITS.FormattingEnabled = true;
            this.cBoxDATABITS.Items.AddRange(new object[] {
            "5",
            "6",
            "7",
            "8"});
            this.cBoxDATABITS.Location = new System.Drawing.Point(133, 110);
            this.cBoxDATABITS.Name = "cBoxDATABITS";
            this.cBoxDATABITS.Size = new System.Drawing.Size(272, 28);
            this.cBoxDATABITS.TabIndex = 6;
            this.cBoxDATABITS.Text = "8";
            // 
            // cBoxBAUDRATE
            // 
            this.cBoxBAUDRATE.FormattingEnabled = true;
            this.cBoxBAUDRATE.Items.AddRange(new object[] {
            "2400",
            "4800",
            "9600"});
            this.cBoxBAUDRATE.Location = new System.Drawing.Point(133, 73);
            this.cBoxBAUDRATE.Name = "cBoxBAUDRATE";
            this.cBoxBAUDRATE.Size = new System.Drawing.Size(272, 28);
            this.cBoxBAUDRATE.TabIndex = 7;
            this.cBoxBAUDRATE.Text = "9600";
            // 
            // cBoxCOMPORT
            // 
            this.cBoxCOMPORT.FormattingEnabled = true;
            this.cBoxCOMPORT.Location = new System.Drawing.Point(133, 36);
            this.cBoxCOMPORT.Name = "cBoxCOMPORT";
            this.cBoxCOMPORT.Size = new System.Drawing.Size(272, 28);
            this.cBoxCOMPORT.TabIndex = 8;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(35, 40);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(92, 20);
            this.label1.TabIndex = 9;
            this.label1.Text = "COM PORT";
            this.label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(16, 91);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(237, 36);
            this.progressBar1.TabIndex = 10;
            this.progressBar1.Click += new System.EventHandler(this.progressBar1_Click);
            // 
            // tBoxDATAOUT
            // 
            this.tBoxDATAOUT.Location = new System.Drawing.Point(445, 131);
            this.tBoxDATAOUT.Multiline = true;
            this.tBoxDATAOUT.Name = "tBoxDATAOUT";
            this.tBoxDATAOUT.Size = new System.Drawing.Size(558, 411);
            this.tBoxDATAOUT.TabIndex = 11;
            this.tBoxDATAOUT.TextChanged += new System.EventHandler(this.tBoxDATAOUT_TextChanged);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.groubox5);
            this.groupBox1.Controls.Add(this.groupBox4);
            this.groupBox1.Controls.Add(this.groupBox3);
            this.groupBox1.Controls.Add(this.label7);
            this.groupBox1.Controls.Add(this.groupBox2);
            this.groupBox1.Controls.Add(this.label5);
            this.groupBox1.Controls.Add(this.btnSENDDATA);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.cBoxCOMPORT);
            this.groupBox1.Controls.Add(this.tBoxDATAOUT);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.cBoxPARITYBITS);
            this.groupBox1.Controls.Add(this.cBoxSTOPBITS);
            this.groupBox1.Controls.Add(this.cBoxDATABITS);
            this.groupBox1.Controls.Add(this.cBoxBAUDRATE);
            this.groupBox1.Location = new System.Drawing.Point(1, 6);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(1039, 705);
            this.groupBox1.TabIndex = 12;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Com Port Control";
            this.groupBox1.Enter += new System.EventHandler(this.groupBox1_Enter);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.btnMOVE);
            this.groupBox4.Controls.Add(this.btnHOME);
            this.groupBox4.Location = new System.Drawing.Point(540, 556);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(358, 132);
            this.groupBox4.TabIndex = 22;
            this.groupBox4.TabStop = false;
            // 
            // btnMOVE
            // 
            this.btnMOVE.Location = new System.Drawing.Point(184, 34);
            this.btnMOVE.Name = "btnMOVE";
            this.btnMOVE.Size = new System.Drawing.Size(162, 68);
            this.btnMOVE.TabIndex = 23;
            this.btnMOVE.Text = "MOTION DEMO";
            this.btnMOVE.UseVisualStyleBackColor = true;
            this.btnMOVE.Click += new System.EventHandler(this.btnMOVE_Click);
            // 
            // btnHOME
            // 
            this.btnHOME.Location = new System.Drawing.Point(10, 35);
            this.btnHOME.Name = "btnHOME";
            this.btnHOME.Size = new System.Drawing.Size(162, 68);
            this.btnHOME.TabIndex = 22;
            this.btnHOME.Text = "HOME ARM";
            this.btnHOME.UseVisualStyleBackColor = true;
            this.btnHOME.Click += new System.EventHandler(this.button1_Click_3);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.a6prog);
            this.groupBox3.Controls.Add(this.a3prog);
            this.groupBox3.Controls.Add(this.a4prog);
            this.groupBox3.Controls.Add(this.a5prog);
            this.groupBox3.Controls.Add(this.a2prog);
            this.groupBox3.Controls.Add(this.a1prog);
            this.groupBox3.Controls.Add(this.label18);
            this.groupBox3.Controls.Add(this.label19);
            this.groupBox3.Controls.Add(this.label11);
            this.groupBox3.Controls.Add(this.label6);
            this.groupBox3.Controls.Add(this.label20);
            this.groupBox3.Controls.Add(this.label12);
            this.groupBox3.Controls.Add(this.label21);
            this.groupBox3.Controls.Add(this.label13);
            this.groupBox3.Controls.Add(this.label22);
            this.groupBox3.Controls.Add(this.label14);
            this.groupBox3.Controls.Add(this.label17);
            this.groupBox3.Controls.Add(this.label15);
            this.groupBox3.Controls.Add(this.label16);
            this.groupBox3.Location = new System.Drawing.Point(23, 411);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(400, 278);
            this.groupBox3.TabIndex = 13;
            this.groupBox3.TabStop = false;
            // 
            // label18
            // 
            this.label18.AutoSize = true;
            this.label18.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label18.Location = new System.Drawing.Point(75, 223);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(45, 25);
            this.label18.TabIndex = 34;
            this.label18.Text = "000";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label19.Location = new System.Drawing.Point(27, 223);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(43, 25);
            this.label19.TabIndex = 33;
            this.label19.Text = "A6:";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label11.Location = new System.Drawing.Point(75, 75);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(45, 25);
            this.label11.TabIndex = 23;
            this.label11.Text = "000";
            this.label11.Click += new System.EventHandler(this.label11_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.Location = new System.Drawing.Point(49, 22);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(294, 25);
            this.label6.TabIndex = 18;
            this.label6.Text = "Absolute Arm Position (degrees)";
            this.label6.Click += new System.EventHandler(this.label6_Click);
            // 
            // label20
            // 
            this.label20.AutoSize = true;
            this.label20.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label20.Location = new System.Drawing.Point(75, 194);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(45, 25);
            this.label20.TabIndex = 32;
            this.label20.Text = "000";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label12.Location = new System.Drawing.Point(26, 75);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(43, 25);
            this.label12.TabIndex = 24;
            this.label12.Text = "A1:";
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label21.Location = new System.Drawing.Point(27, 194);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(43, 25);
            this.label21.TabIndex = 31;
            this.label21.Text = "A5:";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label13.Location = new System.Drawing.Point(27, 106);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(43, 25);
            this.label13.TabIndex = 25;
            this.label13.Text = "A2:";
            // 
            // label22
            // 
            this.label22.AutoSize = true;
            this.label22.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label22.Location = new System.Drawing.Point(75, 166);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(45, 25);
            this.label22.TabIndex = 30;
            this.label22.Text = "000";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label14.Location = new System.Drawing.Point(75, 106);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(45, 25);
            this.label14.TabIndex = 26;
            this.label14.Text = "000";
            // 
            // label17
            // 
            this.label17.AutoSize = true;
            this.label17.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label17.Location = new System.Drawing.Point(27, 166);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(43, 25);
            this.label17.TabIndex = 29;
            this.label17.Text = "A4:";
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label15.Location = new System.Drawing.Point(27, 137);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(43, 25);
            this.label15.TabIndex = 27;
            this.label15.Text = "A3:";
            this.label15.Click += new System.EventHandler(this.label15_Click);
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label16.Location = new System.Drawing.Point(75, 136);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(45, 25);
            this.label16.TabIndex = 28;
            this.label16.Text = "000";
            this.label16.Click += new System.EventHandler(this.label16_Click);
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Microsoft Sans Serif", 16F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.Location = new System.Drawing.Point(438, 50);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(235, 37);
            this.label7.TabIndex = 19;
            this.label7.Text = "Command Line";
            this.label7.Click += new System.EventHandler(this.label7_Click);
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.btnOPEN);
            this.groupBox2.Controls.Add(this.progressBar1);
            this.groupBox2.Controls.Add(this.btnCLOSE);
            this.groupBox2.Location = new System.Drawing.Point(23, 238);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(271, 144);
            this.groupBox2.TabIndex = 16;
            this.groupBox2.TabStop = false;
            // 
            // btnOPEN
            // 
            this.btnOPEN.Location = new System.Drawing.Point(16, 25);
            this.btnOPEN.Name = "btnOPEN";
            this.btnOPEN.Size = new System.Drawing.Size(107, 47);
            this.btnOPEN.TabIndex = 3;
            this.btnOPEN.Text = "OPEN";
            this.btnOPEN.UseVisualStyleBackColor = true;
            this.btnOPEN.Click += new System.EventHandler(this.btnOPEN_Click);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(27, 76);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(102, 20);
            this.label5.TabIndex = 15;
            this.label5.Text = "BAUD RATE";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(35, 114);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(92, 20);
            this.label4.TabIndex = 14;
            this.label4.Text = "DATA BITS";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(35, 151);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(91, 20);
            this.label3.TabIndex = 13;
            this.label3.Text = "STOP BITS";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(19, 187);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(107, 20);
            this.label2.TabIndex = 12;
            this.label2.Text = "PARITY BITS";
            // 
            // serialPort1
            // 
            this.serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.serialPort1_DataReceived);
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // btnINC
            // 
            this.btnINC.AutoSize = true;
            this.btnINC.Location = new System.Drawing.Point(15, 25);
            this.btnINC.Name = "btnINC";
            this.btnINC.Size = new System.Drawing.Size(118, 24);
            this.btnINC.TabIndex = 23;
            this.btnINC.TabStop = true;
            this.btnINC.Text = "Incremental";
            this.btnINC.UseVisualStyleBackColor = true;
            this.btnINC.CheckedChanged += new System.EventHandler(this.btnINC_CheckedChanged);
            // 
            // btnABS
            // 
            this.btnABS.AutoSize = true;
            this.btnABS.Location = new System.Drawing.Point(15, 55);
            this.btnABS.Name = "btnABS";
            this.btnABS.Size = new System.Drawing.Size(97, 24);
            this.btnABS.TabIndex = 24;
            this.btnABS.TabStop = true;
            this.btnABS.Text = "Absolute";
            this.btnABS.UseVisualStyleBackColor = true;
            this.btnABS.CheckedChanged += new System.EventHandler(this.btnABS_CheckedChanged);
            // 
            // groubox5
            // 
            this.groubox5.Controls.Add(this.btnINC);
            this.groubox5.Controls.Add(this.btnABS);
            this.groubox5.Location = new System.Drawing.Point(686, 25);
            this.groubox5.Name = "groubox5";
            this.groubox5.Size = new System.Drawing.Size(200, 100);
            this.groubox5.TabIndex = 25;
            this.groubox5.TabStop = false;
            this.groubox5.Text = "Mode";
            // 
            // a1prog
            // 
            this.a1prog.Location = new System.Drawing.Point(159, 77);
            this.a1prog.Name = "a1prog";
            this.a1prog.Size = new System.Drawing.Size(193, 23);
            this.a1prog.TabIndex = 35;
            // 
            // a2prog
            // 
            this.a2prog.Location = new System.Drawing.Point(159, 108);
            this.a2prog.Name = "a2prog";
            this.a2prog.Size = new System.Drawing.Size(193, 23);
            this.a2prog.TabIndex = 36;
            // 
            // a5prog
            // 
            this.a5prog.Location = new System.Drawing.Point(159, 195);
            this.a5prog.Name = "a5prog";
            this.a5prog.Size = new System.Drawing.Size(193, 23);
            this.a5prog.TabIndex = 37;
            // 
            // a4prog
            // 
            this.a4prog.Location = new System.Drawing.Point(159, 166);
            this.a4prog.Name = "a4prog";
            this.a4prog.Size = new System.Drawing.Size(193, 23);
            this.a4prog.TabIndex = 38;
            // 
            // a3prog
            // 
            this.a3prog.Location = new System.Drawing.Point(159, 137);
            this.a3prog.Name = "a3prog";
            this.a3prog.Size = new System.Drawing.Size(193, 23);
            this.a3prog.TabIndex = 39;
            // 
            // a6prog
            // 
            this.a6prog.Location = new System.Drawing.Point(159, 225);
            this.a6prog.Name = "a6prog";
            this.a6prog.Size = new System.Drawing.Size(193, 23);
            this.a6prog.TabIndex = 40;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1050, 712);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.webBrowser1);
            this.Name = "Form1";
            this.Text = " ";
            this.Load += new System.EventHandler(this.Form1_Load_1);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groubox5.ResumeLayout(false);
            this.groubox5.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.WebBrowser webBrowser1;
        private System.Windows.Forms.Button btnCLOSE;
        private System.Windows.Forms.Button btnSENDDATA;
        private System.Windows.Forms.ComboBox cBoxPARITYBITS;
        private System.Windows.Forms.ComboBox cBoxSTOPBITS;
        private System.Windows.Forms.ComboBox cBoxDATABITS;
        private System.Windows.Forms.ComboBox cBoxBAUDRATE;
        private System.Windows.Forms.ComboBox cBoxCOMPORT;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.TextBox tBoxDATAOUT;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Button btnOPEN;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Button btnHOME;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Button btnMOVE;
        private System.Windows.Forms.GroupBox groubox5;
        private System.Windows.Forms.RadioButton btnINC;
        private System.Windows.Forms.RadioButton btnABS;
        private System.Windows.Forms.ProgressBar a6prog;
        private System.Windows.Forms.ProgressBar a3prog;
        private System.Windows.Forms.ProgressBar a4prog;
        private System.Windows.Forms.ProgressBar a5prog;
        private System.Windows.Forms.ProgressBar a2prog;
        private System.Windows.Forms.ProgressBar a1prog;
    }
}

