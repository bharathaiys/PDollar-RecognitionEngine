import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.FileNotFoundException;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

class Point{
    double X;
    double Y;
    int ID;
    public Point(double x,double y,int id)
    {   X=x;
        Y=y;
        ID=id;
    }
}

class PointCloud
{
  public ArrayList<Point> Points;
  public String Name;
  public Point  Origin = new Point(0.0,0.0,0);

  public PointCloud(String name, ArrayList<Point> points, int numPoints) {

    Name = name;
    Points = points;

    Points = PDollarRecognizer.Resample(Points, numPoints);
    Points = PDollarRecognizer.Scale(Points);
    Points = PDollarRecognizer.TranslateTo(Points, Origin);
  }
}


class Result {
     String Name;
     double Score;
     String Time;
  
    public Result(String name, double score)
      { Name = name; Score = score; }
    public Result(String name, double score, String time) 
      { Name = name; Score = score; Time = time; }
  }


/*const NumPointClouds = 16;
const NumPoints = 32;
const Origin = new Point(0,0,0);*/
class PDollarRecognizer {

  static int NumPoints = 32;
  static Point  Origin = new Point(0.0,0.0,0);
  static ArrayList<PointCloud> NumPntClouds = new ArrayList<PointCloud>();

        public PDollarRecognizer() {  }

  public Result Recognize(ArrayList<Point> points)
  {
    PointCloud foundPointCloud = null;
                points = Resample(points, NumPoints);
                points = Scale(points);
                points = TranslateTo(points, Origin);

                double score = Double.POSITIVE_INFINITY;
                for( int i = 0; i < NumPntClouds.size(); i++ ) // for each point-cloud template
                {
                        double distScore = GreedyCloudMatch(points, NumPntClouds.get(i));
                        if( distScore < score ) {
                                score = distScore; // best (least) distance
                                foundPointCloud = NumPntClouds.get(i); // point-cloud
                        }
                }
                return( foundPointCloud== null ) ? new Result("No match.", 0.0)
      : new Result(foundPointCloud.Name, Math.max((score - 2.0) / -2.0, 0.0), String.format("score %f\n", score));
        }

  public int addGesture(String name, ArrayList<Point> points)
  {
    NumPntClouds.add(new PointCloud(name, points, NumPoints));
    int num = 0;
    for( int i = 0; i < NumPntClouds.size(); i++ )
    {
      if( NumPntClouds.get(i).Name.equals( name) )
        num++;
    }
    return num;
  }

  private static double GreedyCloudMatch(ArrayList<Point> points, PointCloud pntCloud)
  {
    double e = 0.50;
    double step = Math.floor(Math.pow(points.size(), 1 - e));

    double min = Double.POSITIVE_INFINITY;
    for( int  i = 0; i < points.size(); i += step )
    {
      double d1 = CloudDistance(points, pntCloud.Points, i);
      double d2 = CloudDistance(pntCloud.Points, points, i);
      min = Math.min(min, Math.min(d1, d2)); // min3
    }
    return min;
  }

  private static double CloudDistance(ArrayList<Point> pts1, ArrayList<Point> pts2, int start)
  {
    // pts1.size() == pts2.size()
    boolean[] matched = new boolean[pts1.size()]; 
    for( int k = 0; k < pts1.size(); k++ )
      matched[k] = false;
    double sum = 0;
    int i = start;
    do
    {
      int index = -1;
      double min = Double.POSITIVE_INFINITY;
      for( int j = 0; j < matched.length; j++ )
      {
        if( !matched[j] ) {
          double d = EuclideanDistance(pts1.get(i), pts2.get(j));
          if( d < min ) {
            min = d;
            index = j;
          }
        }
      }
      matched[index] = true;
      double weight = 1 - ((i - start + pts1.size()) % pts1.size()) / pts1.size();
      sum += weight * min;
      i = (i + 1) % pts1.size();
    } while( i != start );
    return sum;
  }

  public static ArrayList<Point> Resample(ArrayList<Point> points, int n)
  {
    double I = PathLength(points) / (n - 1); // interval length
    double D = 0.0;

    ArrayList<Point> newpoints = new ArrayList<Point>(); 
    newpoints.add(points.get(0));

    for( int i = 1; i < points.size(); i++ )
    {
      if( points.get(i).ID == points.get(i-1).ID )
      {
        double d = EuclideanDistance(points.get(i - 1), points.get(i));
        if ((D + d) >= I)
        {
          double qx = points.get(i - 1).X + ((I - D) / d) * (points.get(i).X - points.get(i - 1).X);
          double qy = points.get(i - 1).Y + ((I - D) / d) * (points.get(i).Y - points.get(i - 1).Y);
          Point q = new Point(qx, qy, points.get(i).ID);
          newpoints.add(q); // append new point 'q'
          points.add(i, q); // insert 'q' at position i in points s.t. 'q' will be the next i
          D = 0.0;
        } else {
          D += d;
        }
      }
    }

    // sometimes we fall a rounding-error short of
    // adding the last point, so add it if so
    if( newpoints.size() == n - 1 ) 
      newpoints.add(new Point(points.get(points.size() - 1).X, points.get(points.size() - 1).Y, points.get(points.size() - 1).ID));
    return newpoints;
  }

  public static ArrayList<Point> Scale(ArrayList<Point> points)
  {
    double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
    for( int i = 0; i < points.size(); i++ ) {
      minX = Math.min(minX, points.get(i).X);
      minY = Math.min(minY, points.get(i).Y);
      maxX = Math.max(maxX, points.get(i).X);
      maxY = Math.max(maxY, points.get(i).Y);
    }

    double size = Math.max(maxX - minX, maxY - minY);
    ArrayList<Point> newpoints = new ArrayList<Point>();

    for( int i = 0; i < points.size(); i++ ) {
      double qx = (points.get(i).X - minX) / size;
      double qy = (points.get(i).Y - minY) / size;
      newpoints.add(new Point(qx, qy, points.get(i).ID));
    }
    return newpoints;
  }

  public static ArrayList<Point> TranslateTo(ArrayList<Point> points, Point pt) // translates points' centroid
  {
    Point c = Centroid(points);
    ArrayList<Point> newpoints = new ArrayList<Point>();
    for( int i = 0; i < points.size(); i++ ) {
      double qx = points.get(i).X + pt.X - c.X;
      double qy = points.get(i).Y + pt.Y - c.Y;
      newpoints.add(new Point(qx, qy, points.get(i).ID));
    }
    return newpoints;
  }

  private static Point Centroid(ArrayList<Point> points)
  {
    double x = 0.0;
    double y = 0.0;
    for( int i = 0; i < points.size(); i++ ) {
      x += points.get(i).X;
      y += points.get(i).Y;
    }
    x /= points.size();
    y /= points.size();
    return new Point(x, y, 0);
  }

  // average distance between corresponding points in two paths
  private static double PathDistance(ArrayList<Point> pts1, ArrayList<Point> pts2)
  {
    double d = 0.0;
    for( int i = 0; i < pts1.size(); i++ ) // assumes pts1.size() == pts2.size()
      d += EuclideanDistance(pts1.get(i), pts2.get(i));
    return d / pts1.size();
  }

  // length traversed by a point path
  private static double PathLength(ArrayList<Point> points)
  {
    double d = 0.0;
    for( int i = 1; i < points.size(); i++ )
    {
      if( points.get(i).ID == points.get(i-1).ID )
        d += EuclideanDistance(points.get(i - 1), points.get(i));
    }
    return d;
  }

  // Euclidean distance between two points
  private static double EuclideanDistance(Point p1, Point p2)
  {
    double dx = p2.X - p1.X;
    double dy = p2.Y - p1.Y;
    return Math.sqrt(dx * dx + dy * dy);
  }
}
  //
  // PointCloud class: a point-cloud template
  //

  
  
    //
    // PointCloud class: a point-cloud template
    //
    public class pdollar{

        public static void HelpScreen(){
          System.out.println("Help Screen\n");
          System.out.println("pdollar -t <gesturefile>");
          System.out.println("Adds the gesture file to the list of getsure templates\nEg. pdollar -t exclamation_point.txt\n");
          System.out.println("pdollar -r");
          System.out.println("Clears the templates\nEg. pdollar -r\n");
          System.out.println("pdollar <eventstream>");
          System.out.println("Prints the name of gestures as they are recognized from the event stream\nEg. pdollar exclamation_point_eventfile.txt\n");     
        }
    
        public static void add(String args) throws FileNotFoundException, IOException{
              int id=0;
              FileReader rdr = new FileReader(args);
              FileWriter wrtr = new FileWriter("gesture_template.txt", true);
              BufferedReader br = new BufferedReader(rdr);
              BufferedWriter bw = new BufferedWriter(wrtr);
              
              bw.write("seperation");
              bw.newLine();      
              bw.write(br.readLine());
              bw.newLine();
              String s;
    
              while((s=br.readLine()) != null) {
                     if(s.equals("BEGIN")){   
                     id++;              
                         continue;
                     }
                     else if(s.equals("END")){
                        
                        continue;
                      }
    
                    else{
                         bw.write(s + "," + Integer.toString(id));
                         bw.newLine();
                     }  
              } 
              bw.write("END");
              bw.newLine();
              bw.flush();
              bw.close();
              System.out.println("Gesture file has been added");
    
        }
    
        public static void clrGTemplate() throws FileNotFoundException{
          File file  = new File("gesture_template.txt");
          if(file.exists()){
            PrintWriter wrtr = new PrintWriter(file);
            wrtr.print("");
            System.out.println("File cleared");
          }
        }
    
    
        public static void gesturepoints(PDollarRecognizer recog_obj) throws IOException
        {
               ArrayList<Point> pnts = new ArrayList<Point>();
               FileReader rdr = new FileReader("gesture_template.txt");
            BufferedReader br = new BufferedReader(rdr);
            String s,ges_name="";
            
            while((s = br.readLine()) != null)
            {
                
                if(s.equals("seperation")) {
                    ges_name = br.readLine();
                    continue;
                }
                else if(s.equals("END"))
                {
                    recog_obj.addGesture(ges_name,pnts); 
    
                    pnts.clear(); 
                    ges_name ="";
                }
                else {   
                    String cd[] = s.split(",");
                    Point pt = new Point( Double.parseDouble(cd[0]), Double.parseDouble(cd[1]), Integer.parseInt(cd[2]));
                    pnts.add(pt); 
                }
            }
            br.close();
        }
    
    
    public static void printEventstream(String args) throws IOException{
              
              FileReader reader = new FileReader(args);
              BufferedReader br = new BufferedReader(reader);
              ArrayList<Point> pt_list = new ArrayList<Point>();
              PDollarRecognizer recog_obj = new PDollarRecognizer();
              gesturepoints(recog_obj);
              int num = 0;
               
               String s;
               while((s= br.readLine()) != null) {
                   if(s.equals("MOUSEDOWN")){
                       num=num+1;
                       continue;
                   }
                   else if(s.equals("MOUSEUP")){
                       
                       continue;    
                   }
                   else if(s.equals("RECOGNIZE")){
                      
                       Result result = recog_obj.Recognize(pt_list);
                       System.out.println(result.Name);
                       pt_list.clear();
                       result = null;
    
                   }
                   else{
                       String cd[] = s.split(",");
    
                       Point pt = new Point(Double.parseDouble(cd[0]),Double.parseDouble(cd[1]),num);
                       pt_list.add(pt);
                   }    
               }
    } 
        
      
      
    public static void main(String[] args) throws IOException{
     
          if(args.length==0)
            HelpScreen();              
              
          
          else {
            if(args[0].equals("-t") && args[1] != null)
              add(args[1]);  
             
            else if(args[0].equals("-r"))
              clrGTemplate();                 
    
            else if(args[0] != null && args.length == 1)
                 printEventstream(args[0]);           
                
            else
               System.out.println("Command not Valid");
        
          } 
      }
    }
    
    
