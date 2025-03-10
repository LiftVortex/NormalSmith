using System.Diagnostics;
using System.Reflection;
using System.Windows;
using System.Windows.Navigation;

namespace NormalSmith
{
    public partial class CreditsWindow : Window
    {
        public CreditsWindow()
        {
            InitializeComponent();
            string newTitle = Assembly.GetExecutingAssembly().GetName().Version.ToString();
            CreditsTitle.Text = "Normal Smith v" + newTitle.Remove(newTitle.Length - 2);
        }

        // This handler makes sure the hyperlink opens in the default browser.
        private void Hyperlink_RequestNavigate(object sender, RequestNavigateEventArgs e)
        {
            Process.Start(new ProcessStartInfo(e.Uri.AbsoluteUri) { UseShellExecute = true });
            e.Handled = true;
        }
    }
}
