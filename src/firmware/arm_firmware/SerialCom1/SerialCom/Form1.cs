#define CRT_SECURE_NO_WARNINGS
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;




namespace SerialCom
{

    public partial class Form1 : Form
    {

        bool inc_flag;
        bool abs_flag;

        const int a1_max = 360;
        const int a2_max = 360;
        const int a3_max = 90;
        const int a4_max = 90;
        const int a5_max = 90;
        const int a6_max = 90;

        string dataOUT;
        string homeOUT = "H";
        string moveOUT = "M";
        string inc = "I";
        string abs = "A";
        string dataIN;

        string A1; //angles
        string A2;
        string A3;
        string A4;
        string A5;
        string A6;

        public Form1()
        {
            InitializeComponent();
        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void progressBar1_Click(object sender, EventArgs e)
        {

        }

        

        private void button1_Click(object sender, EventArgs e)
        {

        }


        private void webBrowser1_DocumentCompleted(object sender, WebBrowserDocumentCompletedEventArgs e)
        {

        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        private void tBoxDATAOUT_TextChanged(object sender, EventArgs e)
        {

        }

        private void Form1_Load_1(object sender, EventArgs e)
        {
            string[] ports = SerialPort.GetPortNames();
            cBoxCOMPORT.Items.AddRange(ports);
        }

        private void btnOPEN_Click(object sender, EventArgs e)
        {

            try
            {
                serialPort1.PortName = cBoxCOMPORT.Text;
                serialPort1.BaudRate = Convert.ToInt32(cBoxBAUDRATE.Text);
                serialPort1.DataBits = Convert.ToInt32(cBoxDATABITS.Text);
                serialPort1.StopBits = (StopBits)Enum.Parse(typeof(StopBits), cBoxSTOPBITS.Text);
                serialPort1.Parity = (Parity)Enum.Parse(typeof(Parity), cBoxPARITYBITS.Text);

                serialPort1.Open();
                progressBar1.Value = 100;
            }
            catch (Exception err)
            {
                MessageBox.Show(err.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void button2_Click(object sender, EventArgs e) /* Close button */
        {
            if(serialPort1.IsOpen)
            {
                serialPort1.Close();
                progressBar1.Value = 0;
            }
        }


        private void button3_Click(object sender, EventArgs e) /* send data button*/
        {
            if (serialPort1.IsOpen)
            {
                dataOUT = tBoxDATAOUT.Text;

                if (abs_flag)
                {
                    dataOUT = abs + dataOUT;
                }

                else if(inc_flag)
                {
                    dataOUT = inc + dataOUT;
                }
                
                serialPort1.WriteLine(inc+dataOUT);
                tBoxDATAOUT.Text = "";
            }    
        }

        private void label7_Click(object sender, EventArgs e)
        {

        }

        private void label6_Click(object sender, EventArgs e)
        {

        }

        private void absPosition_TextChanged(object sender, EventArgs e)
        {

        }

      

        private void label10_Click(object sender, EventArgs e)
        {

        }

        private void label11_Click(object sender, EventArgs e)
        {
            
        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (serialPort1.ReadLine().Contains('r'))
            {
                A1 = serialPort1.ReadLine();
                A2 = serialPort1.ReadLine();
                A3 = serialPort1.ReadLine();
                A4 = serialPort1.ReadLine();
                A5 = serialPort1.ReadLine();
                A6 = serialPort1.ReadLine();
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            label11.Text = A1;
            label14.Text = A2;
            label16.Text = A3;
            label22.Text = A4;
            label20.Text = A5;
            label18.Text = A6;

            if (A1 != null)
            {
                a1prog.Value = (int)((100 * Int64.Parse(A1)) / a1_max);
            }
            if (A2 != null)
            {
                a2prog.Value = (int)((100 * Int64.Parse(A2)) / a2_max);
            }
            if (A3 != null)
            {
                a3prog.Value = (int)((100 * Int64.Parse(A3)) / a3_max);
            }
            if (A4 != null)
            {
                a4prog.Value = (int)((100 * Int64.Parse(A4)) / a4_max);
            }
            if (A5 != null)
            {
                a5prog.Value = (int)((100 * Int64.Parse(A5)) / a5_max);
            }
            if (A6 != null)
            {
                a6prog.Value = (int)((100 * Int64.Parse(A6)) / a6_max);
            }

        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            
        }

        private void button1_Click_2(object sender, EventArgs e)
        {
          
        }

        private void label15_Click(object sender, EventArgs e)
        {

        }

        private void label16_Click(object sender, EventArgs e)
        {

        }

        private void button1_Click_3(object sender, EventArgs e) // HOME BUTTON // 
        {
            serialPort1.WriteLine(homeOUT);
        }

        private void btnMOVE_Click(object sender, EventArgs e) // MOTION DEMO BUTTON // 
        {
            serialPort1.WriteLine(moveOUT);
        }

        private void btnINC_CheckedChanged(object sender, EventArgs e)
        {
            inc_flag = !inc_flag;
        }

        private void btnABS_CheckedChanged(object sender, EventArgs e)
        {
            abs_flag = !abs_flag;
        }
    }
}
