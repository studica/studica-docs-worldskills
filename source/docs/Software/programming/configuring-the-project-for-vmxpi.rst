Configuring the project for VMXpi
=================================

This guide will show the steps required to configure the project to be deployed to the VMXpi
      
VS Code Changes
---------------
      
In VS Code the ``build.gradle`` file needs to be changed or replaced to match the code in the tabs below. This will allow gradle to build and deploy for the VMXpi as a target. 

.. hint:: Only the highlighted lines must be changed

.. tabs:: 
        
   .. tab:: Java 
   
      .. code-block:: groovy
         :linenos:
         :emphasize-lines: 3,15,47,52

         plugins {
             id "java"
             id "com.kauailabs.first.GradleRIO" version "2020.3.2-beta-1"
         }

         sourceCompatibility = JavaVersion.VERSION_11
         targetCompatibility = JavaVersion.VERSION_11

         def ROBOT_MAIN_CLASS = "frc.robot.Main"

         // Define my targets (RoboRIO) and artifacts (deployable files)
         // This is added by GradleRIO's backing project EmbeddedTools.
         deploy {
             targets {
                 vmxpi("roborio") {
                     // Team number is loaded either from the .wpilib/wpilib_preferences.json
                     // or from command line. If not found an exception will be thrown.
                     // You can use getTeamOrDefault(team) instead of getTeamNumber if you
                     // want to store a team number in this file.
                     team = frc.getTeamNumber()
                 }
             }
             artifacts {
                 frcJavaArtifact('frcJava') {
                     targets << "roborio"
                     // Debug can be overridden by command line, for use with VSCode
                     debug = frc.getDebugOrDefault(false)
                 }
                 // Built in artifact to deploy arbitrary files to the roboRIO.
                 fileTreeArtifact('frcStaticFileDeploy') {
                     // The directory below is the local directory to deploy
                     files = fileTree(dir: 'src/main/deploy')
                     // Deploy to RoboRIO target, into /home/lvuser/deploy
                     targets << "roborio"
                     directory = '/home/lvuser/deploy'
                 }
             }
         }

         // Set this to true to enable desktop support.
         def includeDesktopSupport = false

         // Defining my dependencies. In this case, WPILib (+ friends), and vendor libraries.
         // Also defines JUnit 4.
         dependencies {
             implementation wpi.deps.wpilib()
             nativeZip wpi.deps.wpilibJni(wpi.platforms.raspbian)
             nativeDesktopZip wpi.deps.wpilibJni(wpi.platforms.desktop)


             implementation wpi.deps.vendor.java()
             nativeZip wpi.deps.vendor.jni(wpi.platforms.raspbian)
             nativeDesktopZip wpi.deps.vendor.jni(wpi.platforms.desktop)

             testImplementation 'junit:junit:4.12'

             // Enable simulation gui support. Must check the box in vscode to enable support
             // upon debugging
             simulation wpi.deps.sim.gui(wpi.platforms.desktop, false)
         }

         // Setting up my Jar File. In this case, adding all libraries into the main jar ('fat jar')
         // in order to make them all available at runtime. Also adding the manifest so WPILib
         // knows where to look for our Robot Class.
         jar {
             from { configurations.runtimeClasspath.collect { it.isDirectory() ? it : zipTree(it) } }
             manifest edu.wpi.first.gradlerio.GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS)
         }
   
   .. tab:: C++
   
      .. code-block:: groovy
         :linenos:
         :emphasize-lines: 4,11,55
         
         plugins {
             id "cpp"
             id "google-test-test-suite"
             id "com.kauailabs.first.GradleRIO" version "2020.3.2-beta-1"
         }

         // Define my targets (RoboRIO) and artifacts (deployable files)
         // This is added by GradleRIO's backing project EmbeddedTools.
         deploy {
             targets {
                 vmxpi("roborio") {
                     // Team number is loaded either from the .wpilib/wpilib_preferences.json
                     // or from command line. If not found an exception will be thrown.
                     // You can use getTeamOrDefault(team) instead of getTeamNumber if you
                     // want to store a team number in this file.
                     team = frc.getTeamNumber()
                 }
             }
             artifacts {
                 frcNativeArtifact('frcCpp') {
                     targets << "roborio"
                     component = 'frcUserProgram'
                     // Debug can be overridden by command line, for use with VSCode
                     debug = frc.getDebugOrDefault(false)
                 }
                 // Built in artifact to deploy arbitrary files to the roboRIO.
                 fileTreeArtifact('frcStaticFileDeploy') {
                     // The directory below is the local directory to deploy
                     files = fileTree(dir: 'src/main/deploy')
                     // Deploy to RoboRIO target, into /home/lvuser/deploy
                     targets << "roborio"
                     directory = '/home/lvuser/deploy'
                 }
             }
         }

         // Set this to true to include the src folder in the include directories passed
         // to the compiler. Some eclipse project imports depend on this behavior.
         // We recommend leaving this disabled if possible. Note for eclipse project
         // imports this is enabled by default. For new projects, its disabled
         def includeSrcInIncludeRoot = false

         // Set this to true to enable desktop support.
         def includeDesktopSupport = false

         // Enable simulation gui support. Must check the box in vscode to enable support
         // upon debugging
         dependencies {
             simulation wpi.deps.sim.gui(wpi.platforms.desktop, true)
         }

         model {
             components {
                 frcUserProgram(NativeExecutableSpec) {
                     targetPlatform wpi.platforms.raspbian
                     if (includeDesktopSupport) {
                         targetPlatform wpi.platforms.desktop
                     }

                     sources.cpp {
                         source {
                             srcDir 'src/main/cpp'
                             include '**/*.cpp', '**/*.cc'
                         }
                         exportedHeaders {
                             srcDir 'src/main/include'
                             if (includeSrcInIncludeRoot) {
                                 srcDir 'src/main/cpp'
                             }
                         }
                     }

                     // Defining my dependencies. In this case, WPILib (+ friends), and vendor libraries.
                     wpi.deps.wpilib(it)
                     wpi.deps.vendor.cpp(it)
                 }
             }
             testSuites {
                 frcUserProgramTest(GoogleTestTestSuiteSpec) {
                     testing $.components.frcUserProgram

                     sources.cpp {
                         source {
                             srcDir 'src/test/cpp'
                             include '**/*.cpp'
                         }
                     }

                     wpi.deps.wpilib(it)
                     wpi.deps.googleTest(it)
                     wpi.deps.vendor.cpp(it)
                 }
             }
         }