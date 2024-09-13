using System;
using System.Diagnostics;
using System.IO;
using System.Windows.Forms;

namespace freETarget {
    public partial class frmUpload : Form {
        string filePath = string.Empty;
        frmMainWindow mainWindow;

        public frmUpload(frmMainWindow mainWin) {
            InitializeComponent();
            this.mainWindow = mainWin;
            string com = Properties.Settings.Default.portName;
            string baud = Properties.Settings.Default.baudRate.ToString();
            lblPort.Text = "Port: " + com + " @ " + baud;
        }

        private void btnClose_Click(object sender, EventArgs e) {
            this.Close();
        }

        private void btnSelectFile_Click(object sender, EventArgs e) {
            using (OpenFileDialog openFileDialog = new OpenFileDialog()) {
                openFileDialog.InitialDirectory = ".";
                openFileDialog.Filter = "Binary files (*.bin)|*.bin|All files (*.*)|*.*";
                openFileDialog.FilterIndex = 1;
                openFileDialog.RestoreDirectory = true;

                if (openFileDialog.ShowDialog() == DialogResult.OK) {
                    filePath = openFileDialog.FileName;
                    lblHexFile.Text = filePath;
                    btnUpload.Enabled = true;
                } else {
                    btnUpload.Enabled = false;
                }
            }
        }

        private void btnUpload_Click(object sender, EventArgs e) {
            string esptoolPath = @".sptool\";
            string com = Properties.Settings.Default.portName;
            string baud = Properties.Settings.Default.baudRate.ToString();

            using (Process pProcess = new Process()) {
                pProcess.StartInfo.FileName = "python"; // Assuming esptool.py is run with Python
                pProcess.StartInfo.Arguments = esptoolPath + "esptool.py --chip esp32 --port " + com + " --baud " + baud + " write_flash -z 0x1000 " + filePath;
                pProcess.StartInfo.UseShellExecute = false;
                pProcess.StartInfo.RedirectStandardOutput = true;
                pProcess.StartInfo.RedirectStandardError = true;
                pProcess.StartInfo.WindowStyle = ProcessWindowStyle.Hidden;
                pProcess.StartInfo.CreateNoWindow = true; // Do not display a window

                pProcess.OutputDataReceived += new DataReceivedEventHandler(sortOutputHandler);
                pProcess.ErrorDataReceived += new DataReceivedEventHandler(sortOutputHandler);

                pProcess.Start();

                pProcess.BeginOutputReadLine();
                pProcess.BeginErrorReadLine();

                while (!pProcess.HasExited) {
                    Application.DoEvents(); // This keeps your form responsive by processing events
                }
            }

            Console.WriteLine("ESP32 firmware upload finished");
            mainWindow.displayMessage("Upload firmware finished.", false);
            mainWindow.log("Firmware upload of file " + filePath + " completed.");
        }

        private void sortOutputHandler(object sendingProcess, DataReceivedEventArgs e) {
            Console.WriteLine(e.Data);
            this.BeginInvoke(new MethodInvoker(() => {
                txtUploadConsole.AppendText(e.Data + Environment.NewLine ?? string.Empty);
                mainWindow.log(e.Data);
            }));
        }
    }
}

