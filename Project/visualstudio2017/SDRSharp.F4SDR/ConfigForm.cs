using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace SDRSharp.F4SDR
{
    public partial class ConfigForm : Form
    {
        IO io;
        public ConfigForm(IO io)
        {
            this.io = io;
            InitializeComponent();
            LF_GAIN.Value = io.VGA;
            GAIN_VALUE.Text = LF_GAIN.Value.ToString();
            LF_LNA.Checked = io.VGA_LNA;
            IF_LNA.Checked = io.IF_LNA;
            switch(io.RF_PATH)
            {
                case IO.enum_RFPATH.RF_LNA:
                    RF_LNA.Checked = true;
                    break;
                case IO.enum_RFPATH.RF_BYPASS:
                    RF_Bypass.Checked = true;
                    break;
                case IO.enum_RFPATH.RF_LNA58:
                    RF_LNA58.Checked = true;
                    break;
            }

            if (io.IF_PATH == IO.enum_IFPATH.IF836)
                IF836.Checked = true;
            else
                IF902.Checked = true;
        }

        private void ConfigForm_Load(object sender, EventArgs e)
        {

        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void radioButton5_CheckedChanged(object sender, EventArgs e)
        {
            IF836_CheckedChanged(sender, e);
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            io.VGA_LNA = LF_LNA.Checked;
        }

        private void IF_LNA_CheckedChanged(object sender, EventArgs e)
        {
            io.IF_LNA = IF_LNA.Checked;
        }

        private void LF_GAIN_Scroll(object sender, EventArgs e)
        {
            io.VGA = LF_GAIN.Value;
            GAIN_VALUE.Text = LF_GAIN.Value.ToString();
        }

        private void update_rfpath()
        {
            if (RF_LNA.Checked)
                io.RF_PATH = IO.enum_RFPATH.RF_LNA;
            if (RF_Bypass.Checked)
                io.RF_PATH = IO.enum_RFPATH.RF_BYPASS;
            if (RF_LNA58.Checked)
                io.RF_PATH = IO.enum_RFPATH.RF_LNA58;
        }
        private void RF_LNA_CheckedChanged(object sender, EventArgs e)
        {
            update_rfpath();
        }

        private void RF_Bypass_CheckedChanged(object sender, EventArgs e)
        {
            update_rfpath();
        }

        private void RF_LNA58_CheckedChanged(object sender, EventArgs e)
        {
            update_rfpath();
        }

        private void IF836_CheckedChanged(object sender, EventArgs e)
        {
            if (IF836.Checked)
                io.IF_PATH = IO.enum_IFPATH.IF836;
            else
                io.IF_PATH = IO.enum_IFPATH.IF902;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            button1.Visible = false;
            progressBar1.Visible = true;
            progressBar1.Maximum = 100;
            progressBar1.Minimum = 0;
            for (int i = 0; i < 100; i++)
            {
                progressBar1.Value = i;
                System.Threading.Thread.Sleep(20);
                System.Windows.Forms.Application.DoEvents();
            }
            progressBar1.Visible = false;
            button1.Visible = true;
        }
    }
}
