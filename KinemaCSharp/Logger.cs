using System.IO;
using System.Text;

public static class Logger
{
  //private static StreamWriter wr = new StreamWriter(@"C:\Users\Clemens\Documents\temp\Log.txt");
  private static StreamWriter wr = new StreamWriter(Console.OpenStandardOutput());
  public static void WriteMsg(string msg)
	{
		wr.WriteLine(msg);
	}
  public static void Write(string msg)
	{
    wr.Write(msg);
  }

}
