
# JAR repacking

I only really had a dim recollection of how to do this from hacking on the KGS client nearly two decades ago, but the techniques are still basically the same in 2026.

A JAR is just a ZIP file containing compiled classes (and other stuff).

Instead of rebuilding an entire Java application, you can patch a JAR by taking the source Java files of interest, making your modifications, then using the `javac` compiler to build new `.class` files.  Put these files back in the JAR and you have a modified program. This was basically how Minecraft modding got started, although I hear the process is much less hacky now.

You need to reference the original JAR when doing so that various references can be resolved. So if you want to modify `VGA.java`, this will produce your new class files:

```
javac -cp Digital.jar VGA.java
```

You need to copy the resulting class files back into their original paths. So you can do this work within the directory structure of the unzipped JAR, but if you start modifying a bunch of java files spread out all over this will get unmanageable fast.

A `.java` file belonging to a project of any considerable complexity typically has a `package` reference, from which we can derive the path where the emitted `.class` files should go.

```java
package de.neemann.digital.gui.components.graphics;
```

This corresponds to the path
```
/de/neeman/digital/gui/components/graphics
```
An example of a crude JAR repacking script is provided in /scripts. 

# Portable Java on Windows

I decided about a decade ago that I was never going to have Oracle's default Java installation on my system, with its obnoxious updater and all.
Thankfully, most Java applications can be run in a portable way if you have an appropriate OpenJDK installation.

Digital requires JDK v17 or newer.
Microsoft hosts builds of OpenJDK for windows [here](https://learn.microsoft.com/en-us/java/openjdk/older-releases#openjdk-17).

Making use of this is simple:
  - Download the zip version
  - Unzip it in a directory of your choice
  - Set the user environment variable `JAVA_HOME` to that directory 
  - Add `%JAVA_HOME%\bin` to your `PATH`. (Make sure you remove/replace existing entries)

Now you can run JAR files by just calling something like ```javaw -jar Digital.jar```

If you're wanting to make a portable Windows distribution, you can include an appropriate OpenJDK in a subdirectory. 
The only real issue is size - clocking in at 180-200MB, this adds significant bulk.

There is however a way to create a trimmed down JDK specific to a particular application.

First, with OpenJDK already installed on your system via the steps above, scan the JAR in question like so:

```
jdeps --ignore-missing-deps --print-module-deps Digital.jar
```

This will print out a list of base classes that the application requires.

```java.base,java.desktop,java.prefs,java.sql,jdk.unsupported```

You can then produce a stripped JDK via the following command:

```
jlink.exe ^
  --add-modules java.base,java.desktop,java.prefs,java.sql,jdk.unsupported ^
  --strip-debug ^
  --no-header-files ^
  --no-man-pages ^
  --compress=2 ^
  --output DigitalOpenJDK
```

This will produce a portable OpenJDK in the DigitalOpenJDK directory that should be sufficient to execute the source JAR.

