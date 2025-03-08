using System.Reflection;

public static class VersionHelper
{
    public static string GetAppVersion()
    {
        var attribute = Assembly.GetExecutingAssembly().GetCustomAttribute<AssemblyInformationalVersionAttribute>();
        return attribute?.InformationalVersion ?? "unknown";
    }
}
