using System;

using SDRSharp.Radio;
using System.Threading;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.Diagnostics;

namespace SDRSharp.F4SDR
{
    public class IO :
        IFrontendController, IIQStreamController,
        ITunableSource,
        ISpectrumProvider,
        IFloatingConfigDialogProvider,
        IDisposable
    {

        private ConfigForm _gui = null;

        public enum enum_RFPATH
        {
            RF_LNA = 1,
            RF_BYPASS = 2,
            RF_LNA58 = 4,
        };
        public enum enum_IFPATH
        {
            IF836 = 0,
            IF902 = 1,
        };
        public IO()
        {
            f4sdr_dll_init();
        }        

        ~IO()
        {
            Close();
        }

        public void Open()
        {
        }
        public void Close()
        {
            Stop();
        }
        public void Start(SamplesAvailableDelegate cb)
        {
            mInstance = new Callback(native_cb);
            
            _callback = cb;

            f4sdr_setcb(mInstance);
            f4sdr_open();
            update_rfpath();
            f4sdr_set_gain((byte)(_vga + (_vga_lna ? 0x80 : 0)));

        }
        public void Stop()
        {
            f4sdr_close();
        }

        bool ITunableSource.CanTune
        {
            get
            {
                return true;
            }
        }
        long ITunableSource.MinimumTunableFrequency
        {
            get
            {
                return (long)10e6;
            }
        }
        long ITunableSource.MaximumTunableFrequency
        {
            get
            {
                return (long)8000e6;
            }
        }

        public void Dispose()
        {
            Close();
        }

        public long Frequency
        {
            get
            {
                return _frequency;
            }
            set
            {
                this._frequency = value;
                int tune_result = f4sdr_tune(value);
                Trace.WriteLine("tune : " + tune_result);
            }
        }
        public double Samplerate
        {
            get
            {
                return 10000000;
            }
        }

        public float UsableSpectrumRatio
        {
            get
            {
                return 1.0f;
            }
        }


        // for UI interact
        int _vga = Utils.GetIntSetting("F4SDR.VGA", 20);
        public int VGA
        {
            get
            {
                return _vga;
            }
            set
            {
                _vga = value;
                Utils.SaveSetting("F4SDR.VGA", _vga);
                f4sdr_set_gain((byte)(_vga + (_vga_lna ? 0x80 : 0)));
            }
        }

        public bool VGA_LNA
        {
            get
            {
                return _vga_lna;
            }
            set
            {
                _vga_lna = value;
                Utils.SaveSetting("F4SDR.VGALNA", _vga_lna);
                f4sdr_set_gain((byte)(_vga + (_vga_lna ? 0x80 : 0)));
            }
        }
        bool _vga_lna = Utils.GetBooleanSetting("F4SDR.VGALNA", true);

        enum_RFPATH rfpath = (enum_RFPATH)Utils.GetIntSetting("F4SDR.RFPATH", 0);
        public enum_RFPATH RF_PATH
        {
            get
            {
                return rfpath;
            }
            set
            {
                rfpath = value;
                Utils.SaveSetting("F4SDR.RFPATH", (int)rfpath);
                update_rfpath();
            }
        }

        enum_IFPATH ifpath = (enum_IFPATH)Utils.GetIntSetting("F4SDR.IFPATH", 0);
        public enum_IFPATH IF_PATH
        {
            get
            {
                return ifpath;
            }
            set
            {
                ifpath = value;
                Utils.SaveSetting("F4SDR.IFPATH", (int)ifpath);
                update_rfpath();
            }
        }

        bool iflna = Utils.GetBooleanSetting("F4SDR.IFLNA", true);
        public bool IF_LNA
        {
            get
            {
                return iflna;
            }
            set
            {
                iflna = value;
                Utils.SaveSetting("F4SDR.IFLNA", iflna);
                update_rfpath();
            }
        }

        void update_rfpath()
        {
            int path = ((byte)rfpath << 5) | ((byte)ifpath << 1) | (iflna?0x10:0);
            f4sdr_set_path((byte)path);
        }

        public void ShowSettingGUI(IWin32Window parent)
        {
            if (_gui == null || _gui.IsDisposed)
                _gui = new ConfigForm(this);
            if (!_gui.Visible)
            {
                _gui.Show(parent);
                _gui.Activate();
            }
        }

        public void HideSettingGUI()
        {
            if (_gui != null)
                _gui.Hide();
        }

        // privates
        private Callback mInstance;
        long _frequency = (long)100e6;
        private SDRSharp.Radio.SamplesAvailableDelegate _callback;
        
        // native
        private delegate int Callback(IntPtr ptr, int length);
        private unsafe int native_cb(IntPtr ptr, int length)
        {
            int complex_count = length / sizeof(Complex);
            Complex* c = (Complex*)ptr.ToPointer();
            _callback(this, c, complex_count);
            return 0;
        }



        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_setcb(Callback fn);

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_dll_init();

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_open();

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_close();

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_tune(long freq);

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_set_gain(byte gain_code);

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_set_path(byte path);

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int f4sdr_control_io(bool tx, byte request, ushort value, ushort index, [In, Out] byte[] data, int length);

    }
}
