using System;

using SDRSharp.Radio;
using System.Threading;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace SDRSharp.F4SDR
{
    public class IO :
        IFrontendController, IIQStreamController,
        ITunableSource,
        ISpectrumProvider,
        IFloatingConfigDialogProvider,
        IDisposable
    {
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
                return (long)2400e6;
            }
        }
        long ITunableSource.MaximumTunableFrequency
        {
            get
            {
                return (long)2500e6;
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
            }
        }
        public double Samplerate
        {
            get
            {
                return 5120000;
            }
        }

        public float UsableSpectrumRatio
        {
            get
            {
                return 1.0f;
            }
        }


        public void ShowSettingGUI(IWin32Window parent)
        {
            /*
            _gui = new config(this);
            if (_gui.IsDisposed)
                return;
            _gui.Show(parent);
            _gui.Activate();
            */
        }

        public void HideSettingGUI()
        {
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
        private static extern void f4sdr_setcb(Callback fn);

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void f4sdr_dll_init();

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void f4sdr_open();

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void f4sdr_close();
    }
}
