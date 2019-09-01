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

        public unsafe void worker_thread_run()
        {
            while (isStreaming)
            {
                //_callback(this, _iqPtr, sample_count/10);
                //test_cb();
                Thread.Sleep((int)(256.0 / 1000 * 1024 / 1280));
            }

            Console.WriteLine("EXIT");
        }

        private static void worker_thread_enrty(Object obj)
        {
            IO _this = (IO)obj;
            _this.worker_thread_run();
        }

        ~IO()
        {
            Close();
        }

        public void Open()
        {
        }
        private unsafe void ptr_init()
        {
            _iqBuffer = UnsafeBuffer.Create((int)sample_count, sizeof(Complex));
            _iqPtr = (Complex*)_iqBuffer;
            for (int i = 0; i < sample_count; i++)
            {
                _iqPtr[i].Real = (float)Math.Cos(i * 2 * Math.PI / 100);
                _iqPtr[i].Imag = (float)Math.Sin(i * 2 * Math.PI / 100);
            }
        }
        public void Close()
        {
            Stop();
        }
        public void Start(SamplesAvailableDelegate cb)
        {
            mInstance = new Callback(native_cb);

            ptr_init();
            _callback = cb;
            isStreaming = true;
            dummy_thread = new Thread(worker_thread_enrty);
            dummy_thread.Start(this);

            f4sdr_setcb(mInstance);
            f4sdr_open();
        }
        public void Stop()
        {
            if (!isStreaming)
                return;

            isStreaming = false;
            //dummy_thread.Join();
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


        long _frequency = (long)100e6;
        private SDRSharp.Radio.SamplesAvailableDelegate _callback;
        private unsafe Complex* _iqPtr;
        private UnsafeBuffer _iqBuffer;
        int sample_count = 1000000;
        bool isStreaming = false;
        Thread dummy_thread;
        
        // native
        private delegate int Callback(IntPtr ptr, int length);
        private unsafe int native_cb(IntPtr ptr, int length)
        {
            int complex_count = length / sizeof(Complex);
            Complex* c = (Complex*)ptr.ToPointer();
            _callback(this, c, complex_count);
            return 0;
        }
        private Callback mInstance;



        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void f4sdr_setcb(Callback fn);

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void f4sdr_dll_init();

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void f4sdr_open();

        [DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void f4sdr_close();

        //[DllImport("F4SDRNative.dll", CallingConvention = CallingConvention.Cdecl)]
        //private static extern void test_cb();

    }
}
