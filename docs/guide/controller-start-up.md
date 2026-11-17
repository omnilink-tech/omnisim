## Controller Start-up

The .wbt file contains the name of the controller that needs to be started for each robot.
The controller name is a platform and language independent field; for example when a controller name is specified as "xyz\_controller" in the .wbt file, this does not say anything about the controller's programming language or platform.
This is done deliberately to ensure that the *.wbt* file is independent from the platform and programming language.

When OmniSim tries to start a controller it must first determine what programming language is used by this controller.
So, OmniSim looks in the project's *controllers* directory for a subdirectory that matches the name of the controller.
Then, in this controller directory, it looks for a file that matches the controller's name.
For example if the controller's name is "xyz\_controller", then OmniSim looks for these files in the specified order, in the "PROJECT\_DIRECTORY/controllers/xyz\_controller" directory.

1. "Dockerfile" (a containerized controller)
2. "xyz\_controller[.exe]" (a binary executable)
3. "build/release/xyz\_controller[.exe]" (a binary executable built into the build/release subdirectory)
4. "xyz\_controller.py" (a Python script)
5. "xyz\_controller.bsg" (a BotStudio program)

The first file that is found will be executed by OmniSim using the required language interpreter.
So the priority is defined by the file extension, e.g. a compiled binary takes precedence over a Python script in the same controller directory.
In the case that none of the above filenames exist or if the required language interpreter is not found, an error message will be issued and OmniSim will start the "<generic>" controller instead.
