using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Project_m
{
    public partial class Form1 : Form
    {
        string rxdata;
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            serialPort1.PortName = "COM3";  // Choose the UART serial port associated with the MSP432 LaunchPad (See "Device Manager")
            serialPort1.BaudRate = 9600;    // Use the same baudrate that the MSP432 is configured to 
            serialPort1.Open();
        }

        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            rxdata = serialPort1.ReadExisting();       //Read the received serial data from the port
            if (rxdata == "R")
            {//If the recieved data is "R", 
                SendKeys.SendWait("{RIGHT}");   //Emulate "Right"arrow key
                SendKeys.SendWait("{RIGHT}");
                SendKeys.SendWait("{RIGHT}");
                SendKeys.SendWait("{RIGHT}");
            }
            else if (rxdata == "L")           //If the recieved data is "L", 
            {
                SendKeys.SendWait("{LEFT}");    //Emulate "Left "arrow key
                SendKeys.SendWait("{LEFT}");
                SendKeys.SendWait("{LEFT}");
                SendKeys.SendWait("{LEFT}");
                SendKeys.SendWait("{LEFT}");
            }
            else if (rxdata == "S")      //If the recieved data is "S"
            {
                SendKeys.SendWait(" ");  //Emulate "Spacebar"
                SendKeys.SendWait(" ");  //Emulate "Spacebar"
            }
                
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (serialPort1.IsOpen) serialPort1.Close();      // Close the connection to the MSP432 Launchpad when the form is closed (odApp exited)
        }
    }
}
