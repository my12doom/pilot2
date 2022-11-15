namespace SDRSharp.F4SDR
{
    partial class ConfigForm
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
            this.RF_LNA = new System.Windows.Forms.RadioButton();
            this.RF_Bypass = new System.Windows.Forms.RadioButton();
            this.RF_LNA58 = new System.Windows.Forms.RadioButton();
            this.IF836 = new System.Windows.Forms.RadioButton();
            this.IF902 = new System.Windows.Forms.RadioButton();
            this.LF_LNA = new System.Windows.Forms.CheckBox();
            this.label1 = new System.Windows.Forms.Label();
            this.IF_LNA = new System.Windows.Forms.CheckBox();
            this.LF_GAIN = new System.Windows.Forms.TrackBar();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.button1 = new System.Windows.Forms.Button();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.GAIN_VALUE = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.LF_GAIN)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.SuspendLayout();
            // 
            // RF_LNA
            // 
            this.RF_LNA.AutoSize = true;
            this.RF_LNA.Checked = true;
            this.RF_LNA.Location = new System.Drawing.Point(6, 20);
            this.RF_LNA.Name = "RF_LNA";
            this.RF_LNA.Size = new System.Drawing.Size(41, 16);
            this.RF_LNA.TabIndex = 0;
            this.RF_LNA.TabStop = true;
            this.RF_LNA.Text = "LNA";
            this.RF_LNA.UseVisualStyleBackColor = true;
            this.RF_LNA.CheckedChanged += new System.EventHandler(this.RF_LNA_CheckedChanged);
            // 
            // RF_Bypass
            // 
            this.RF_Bypass.AutoSize = true;
            this.RF_Bypass.Location = new System.Drawing.Point(6, 41);
            this.RF_Bypass.Name = "RF_Bypass";
            this.RF_Bypass.Size = new System.Drawing.Size(59, 16);
            this.RF_Bypass.TabIndex = 1;
            this.RF_Bypass.Text = "Bypass";
            this.RF_Bypass.UseVisualStyleBackColor = true;
            this.RF_Bypass.CheckedChanged += new System.EventHandler(this.RF_Bypass_CheckedChanged);
            // 
            // RF_LNA58
            // 
            this.RF_LNA58.AutoSize = true;
            this.RF_LNA58.Location = new System.Drawing.Point(6, 64);
            this.RF_LNA58.Name = "RF_LNA58";
            this.RF_LNA58.Size = new System.Drawing.Size(65, 16);
            this.RF_LNA58.TabIndex = 2;
            this.RF_LNA58.Text = "LNA5.8G";
            this.RF_LNA58.UseVisualStyleBackColor = true;
            this.RF_LNA58.CheckedChanged += new System.EventHandler(this.RF_LNA58_CheckedChanged);
            // 
            // IF836
            // 
            this.IF836.AutoSize = true;
            this.IF836.Location = new System.Drawing.Point(6, 20);
            this.IF836.Name = "IF836";
            this.IF836.Size = new System.Drawing.Size(59, 16);
            this.IF836.TabIndex = 3;
            this.IF836.Text = "836Mhz";
            this.IF836.UseVisualStyleBackColor = true;
            this.IF836.CheckedChanged += new System.EventHandler(this.IF836_CheckedChanged);
            // 
            // IF902
            // 
            this.IF902.AutoSize = true;
            this.IF902.Checked = true;
            this.IF902.Location = new System.Drawing.Point(6, 42);
            this.IF902.Name = "IF902";
            this.IF902.Size = new System.Drawing.Size(59, 16);
            this.IF902.TabIndex = 4;
            this.IF902.TabStop = true;
            this.IF902.Text = "902Mhz";
            this.IF902.UseVisualStyleBackColor = true;
            this.IF902.CheckedChanged += new System.EventHandler(this.radioButton5_CheckedChanged);
            // 
            // LF_LNA
            // 
            this.LF_LNA.AutoSize = true;
            this.LF_LNA.Checked = true;
            this.LF_LNA.CheckState = System.Windows.Forms.CheckState.Checked;
            this.LF_LNA.Location = new System.Drawing.Point(10, 22);
            this.LF_LNA.Name = "LF_LNA";
            this.LF_LNA.Size = new System.Drawing.Size(42, 16);
            this.LF_LNA.TabIndex = 8;
            this.LF_LNA.Text = "LNA";
            this.LF_LNA.UseVisualStyleBackColor = true;
            this.LF_LNA.CheckedChanged += new System.EventHandler(this.checkBox1_CheckedChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(23, 7);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(17, 12);
            this.label1.TabIndex = 9;
            this.label1.Text = "RF";
            this.label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // IF_LNA
            // 
            this.IF_LNA.AutoSize = true;
            this.IF_LNA.Checked = true;
            this.IF_LNA.CheckState = System.Windows.Forms.CheckState.Checked;
            this.IF_LNA.Location = new System.Drawing.Point(6, 64);
            this.IF_LNA.Name = "IF_LNA";
            this.IF_LNA.Size = new System.Drawing.Size(42, 16);
            this.IF_LNA.TabIndex = 11;
            this.IF_LNA.Text = "LNA";
            this.IF_LNA.UseVisualStyleBackColor = true;
            this.IF_LNA.CheckedChanged += new System.EventHandler(this.IF_LNA_CheckedChanged);
            // 
            // LF_GAIN
            // 
            this.LF_GAIN.Location = new System.Drawing.Point(10, 46);
            this.LF_GAIN.Maximum = 127;
            this.LF_GAIN.Minimum = 1;
            this.LF_GAIN.Name = "LF_GAIN";
            this.LF_GAIN.Size = new System.Drawing.Size(205, 45);
            this.LF_GAIN.SmallChange = 10;
            this.LF_GAIN.TabIndex = 13;
            this.LF_GAIN.TickFrequency = 15;
            this.LF_GAIN.Value = 45;
            this.LF_GAIN.Scroll += new System.EventHandler(this.LF_GAIN_Scroll);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.IF_LNA);
            this.groupBox1.Controls.Add(this.IF902);
            this.groupBox1.Controls.Add(this.IF836);
            this.groupBox1.Location = new System.Drawing.Point(128, 7);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(101, 90);
            this.groupBox1.TabIndex = 14;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "1st IF";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.RF_LNA58);
            this.groupBox2.Controls.Add(this.RF_Bypass);
            this.groupBox2.Controls.Add(this.RF_LNA);
            this.groupBox2.Location = new System.Drawing.Point(12, 7);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(99, 89);
            this.groupBox2.TabIndex = 15;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "RF Frontend";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.GAIN_VALUE);
            this.groupBox3.Controls.Add(this.LF_LNA);
            this.groupBox3.Controls.Add(this.LF_GAIN);
            this.groupBox3.Location = new System.Drawing.Point(12, 114);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(217, 97);
            this.groupBox3.TabIndex = 16;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "LF";
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(15, 225);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(113, 22);
            this.button1.TabIndex = 17;
            this.button1.Text = "Load FPGA...";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(12, 224);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(217, 23);
            this.progressBar1.TabIndex = 18;
            this.progressBar1.Visible = false;
            // 
            // GAIN_VALUE
            // 
            this.GAIN_VALUE.AutoSize = true;
            this.GAIN_VALUE.Location = new System.Drawing.Point(128, 23);
            this.GAIN_VALUE.Name = "GAIN_VALUE";
            this.GAIN_VALUE.Size = new System.Drawing.Size(11, 12);
            this.GAIN_VALUE.TabIndex = 14;
            this.GAIN_VALUE.Text = "0";
            // 
            // ConfigForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(243, 255);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.label1);
            this.DoubleBuffered = true;
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "ConfigForm";
            this.Text = "F4SDR";
            this.Load += new System.EventHandler(this.ConfigForm_Load);
            ((System.ComponentModel.ISupportInitialize)(this.LF_GAIN)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.RadioButton RF_LNA;
        private System.Windows.Forms.RadioButton RF_Bypass;
        private System.Windows.Forms.RadioButton RF_LNA58;
        private System.Windows.Forms.RadioButton IF836;
        private System.Windows.Forms.RadioButton IF902;
        private System.Windows.Forms.CheckBox LF_LNA;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.CheckBox IF_LNA;
        private System.Windows.Forms.TrackBar LF_GAIN;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Label GAIN_VALUE;
    }
}