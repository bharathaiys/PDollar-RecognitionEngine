PDollar ($P) Recognition Engine in Java

Bharath Shankar

- Running pdollar with no arguments should print a help screen.

-pdollar should support the following arguments

  pdollar –t <gesturefile>
  Adds the gesture file to the list of gesture templates
  
  pdollar -r
  Clears the templates
  
  pdollar <eventstream>
  Prints the name of gestures as they are recognized from the event stream.

HOW TO RUN:

1. make -f makefile
2. java pdollar

gesturefile format (examples to be provided):
GestureName
BEGIN
x,y <- List of points, a point per line ...
x,y
END

eventstream file format (examples to be provided):
MOUSEDOWN
x,y <- List of points, a point per line
MOUSEUP
RECOGNIZE <- When you see this you should output the result .

Example Run:
$pdollar eventstream.txt CROSS
CIRCLE
$
