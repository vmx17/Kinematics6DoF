using ManipulatorKinematics.ViewModels;

using Microsoft.UI.Xaml.Controls;

namespace ManipulatorKinematics.Views;

public sealed partial class BlankPage : Page
{
    public BlankViewModel ViewModel
    {
        get;
    }

    public BlankPage()
    {
        ViewModel = App.GetService<BlankViewModel>();
        InitializeComponent();
    }
}
