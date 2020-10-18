:%s/^[ ]\+/^I/
JFLAGS = -g
JC = javac
.SUFFIXES: .java .class
.java.class:
	$(JC) $(JFLAGS) $*.java
	jar cvfe pdollar.jar pdollar *.class

CLASSES =	\
	pdollar.java
default:classes

classes:$(CLASSES:.java=.class)

clean:
	$(RM) *.class
