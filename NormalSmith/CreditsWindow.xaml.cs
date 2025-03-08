using System.Diagnostics;
using System.Windows;
using System.Windows.Navigation;

namespace NormalSmith
{
    public partial class CreditsWindow : Window
    {
        public CreditsWindow()
        {
            InitializeComponent();
        }

        // This handler makes sure the hyperlink opens in the default browser.
        private void Hyperlink_RequestNavigate(object sender, RequestNavigateEventArgs e)
        {
            Process.Start(new ProcessStartInfo(e.Uri.AbsoluteUri) { UseShellExecute = true });
            e.Handled = true;
        }
    }
}
