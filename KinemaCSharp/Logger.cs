using System.IO;
using System.Text;

public static class Logger
{
	private static StreamWriter wr = new StreamWriter(@"C:\Temp\Log.txt");

	public static void WriteMsg(string msg)
	{
		wr.WriteLine(msg);
	}
  public static void Write(string msg)
	{
    wr.Write(msg);
  }

}
