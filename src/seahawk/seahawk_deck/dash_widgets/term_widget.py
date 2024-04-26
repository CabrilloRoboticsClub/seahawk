import os
import shlex
from re import search
from collections import deque

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg

class CmdHistory():
    """
    Simple data structure that emulates cyclic bidirectional iteration
    of the terminal command line history. 
    """

    def __init__(self):
        """
        Initialize `CmdHistory` object
        """
        self.history = []
        self.i = None
    
    def next(self) -> str:
        """
        Returns next command
        """
        # Wrap around when i is out or range
        if self.i == len(self.history): self.i = 0
        try:
            self.i += 1
            return self.history[self.i]
        except IndexError or TypeError:
            return ""

    def prev(self) -> str:
        """
        Returns previous command
        """
        # Wrap around when i is out or range
        if self.i == -1: self.i = len(self.history) - 1
        try:
            self.i -= 1
            return self.history[self.i]
        except IndexError or TypeError:
            return ""

    def append(self, cmd: str):
        """
        Append a new command to the list of commands ran
        """
        self.history.append(cmd)
        self.i = len(self.history) # Rest the index so it looks at the most recent command
    

class TermWidget(qtw.QWidget):
    """
    Creates a `TermWidget` which inherits from the `qtw.QWidget` class. A `TermWidget`
    provides an embedded terminal.
    """

    def __init__(self, parent: qtw.QWidget, style_sheet_file: str, colors: dict):
        """
        Initialize terminal widget.
        
        Args:
            parent: Widget to overlay 'NumericDataWidget' on.
            style_sheet_file: Style sheet text file formatted as a CSS f-string.
            colors: Hex codes to color widget with.
        """
        super().__init__(parent)

        with open(style_sheet_file) as style_sheet:
            self.style_sheet = style_sheet.read()

        # The QProcess class is used to start external programs and to communicate with them. 
        self.proc = qtc.QProcess(self)
        self.proc.setProcessChannelMode(qtc.QProcess.MergedChannels)
        self.proc.readyRead.connect(self.read_and_display_cmd_feedback)
        self.proc.setWorkingDirectory(os.getcwd())

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        frame = qtw.QFrame()
        layout_outer.addWidget(frame)

        # Set layout of labels on frame to grid
        layout_inner = qtw.QVBoxLayout(frame)
        frame.setLayout(layout_inner)

        self.cmd_history = CmdHistory()
        
        self.copied_text = ""
        
        # Stores previous commands and feedback to display to the terminal. 
        # Limits number of elements to 200, old elements are discarded.
        # Initial deque contents have to be set to an empty string so
        # it actually accepts strings.
        self.feedback_txt = deque([""], maxlen=200)

        # Feedback window for displaying command results
        self.feedback = qtw.QTextEdit()
        self.feedback.setReadOnly(True)

        # Displays the path of working directory
        self.prompt = qtw.QLabel()
        self.display_prompt()

        # Command line window for the user to enter commands
        self.cmd_line = qtw.QPlainTextEdit()
        self.cmd_line.setLineWrapMode(qtw.QPlainTextEdit.WidgetWidth)
        self.cmd_line.setFocus()
        self.cmd_line.installEventFilter(self)  # Filter command line events (see eventFilter())

        # Add widgets to layout
        layout_inner.addWidget(self.feedback, stretch=98)
        layout_inner.addWidget(self.prompt, stretch=1)
        layout_inner.addWidget(self.cmd_line, stretch=1)
        
        # Apply colors
        self.set_colors(colors)

    def eventFilter(self, a0: qtc.QObject, a1: qtc.QEvent) -> bool:
        """
        Filters and handles events. Makes terminal behave as expected when certain keys are pressed.

        Key press:          Functionality:           
        - return/enter:     Run command
        - up-arrow:         Scroll up in cmd history
        - down-arrow:       Scroll down in cmd history
        - ctrl-c:           Terminate process
        - ctrl-shift-c:     Copy selected text
        - ctrl-shift-v:     Paste text
        - ctrl-l:           Clear screen
        - ctrl-a:           Move to the start of the line
        - ctrl-e:           Move to the end of the line
        - ctrl-b:           Move one character backward
        - ctrl-f:           Move one character forward
        - ctrl-w:           Delete the word before the cursor
        - ctrl-u:           Delete from the cursor to the start of the line
        - ctrl-k:           Delete from the cursor to the end of the line
        """
        if a0 == self.cmd_line: # If the object the event is from is the command line
            if (a1.type() == qtc.QEvent.KeyPress):  # User pressed a key
                key = a1.key()
                # Single key press
                if key == qtc.Qt.Key_Return:    # return/enter: Run command
                    self.run_cmd()
                    return True
                elif key == qtc.Qt.Key_Up:      # up-arrow: Scroll up in cmd history
                    self.cmd_line.setPlainText(self.cmd_history.prev())
                    self.move_cursor(qtg.QTextCursor.End)
                    return True
                elif key == qtc.Qt.Key_Down:    # down-arrow: Scroll down in cmd history
                    self.cmd_line.setPlainText(self.cmd_history.next())
                    self.move_cursor(qtg.QTextCursor.End)
                
                # Key sequence
                seq = qtg.QKeySequence(a1.key() + int(a1.modifiers())) 
                if seq == qtg.QKeySequence("Ctrl+C"):          # ctrl-c: Terminate process
                    if self.proc.state() == qtc.QProcess.Running:
                        self.proc.terminate()
                        self.extend_feedback("Process terminated with ctrl-c")
                    return True
                elif seq == qtg.QKeySequence("Ctrl+Shift+C"):  # ctrl-shift-c: Copy selected text
                    temp_cursor = self.cmd_line.textCursor()
                    self.copied_text = temp_cursor.selectedText()
                    return True
                elif seq == qtg.QKeySequence("Ctrl+Shift+V"):  # ctrl-shift-v: Paste text
                    self.cmd_line.insertPlainText(self.copied_text)
                    self.move_cursor(qtg.QTextCursor.End)
                    return True
                elif seq == qtg.QKeySequence("Ctrl+L"):        # ctrl-l: Clear screen
                    self.run_cmd("clear")
                    return True
                elif seq == qtg.QKeySequence("Ctrl+A"):        # ctrl-a: Move to the start of the line
                    self.move_cursor(qtg.QTextCursor.StartOfLine)
                    return True
                elif seq == qtg.QKeySequence("Ctrl+E"):        # ctrl-e: Move to the end of the line
                    self.move_cursor(qtg.QTextCursor.End)
                    return True
                elif seq == qtg.QKeySequence("Ctrl+B"):        # ctrl-b: Move one character backward
                    self.move_cursor(qtg.QTextCursor.PreviousCharacter)
                    return True
                elif seq == qtg.QKeySequence("Ctrl+F"):        # ctrl-f: Move one character forward
                    self.move_cursor(qtg.QTextCursor.NextCharacter)
                    return True
                elif seq == qtg.QKeySequence("Ctrl+W"):        # ctrl-w: Delete the word before the cursor
                    self.move_cursor(qtg.QTextCursor.PreviousWord, 1, delete=True)
                    return True
                elif seq == qtg.QKeySequence("Ctrl+U"):        # ctrl-u: Delete from the cursor to the start of the line
                    self.move_cursor(qtg.QTextCursor.StartOfLine, 1, delete=True)
                    return True
                elif seq == qtg.QKeySequence("Ctrl+K"):        # ctrl-k: Delete from the cursor to the end of the line
                    self.move_cursor(qtg.QTextCursor.End, 1, delete=True)
                    return True   
        return False

    def run_cmd(self, command: str=None):
        """
        Runs the command provided by the argument or present in the command line. The command
        ran and its results are appended to the feedback window, with formatting. If the
        command is not found, the user is notified with a message. Command is appended
        to the command history list.

        Args:
            command: Command to run (optional).
        """
        self.feedback.setFocus()
        # If provided an argument, use that, otherwise grab the text from the command line
        cmd_txt = command if command else self.cmd_line.toPlainText()
        # Split `cmd_txt` into commands with shell-like syntax
        cmd_list = shlex.split(cmd_txt, posix=False)
        cmd = str(cmd_list[0]).casefold()   # First index holds the actual command
        cmd_args = " ".join(cmd_list[1:])   # The rest of the list contains the arguments

        # HTML formatting strings for colored text
        WARNING     = '<span style="color:#fc3019;">{}</span>'
        SUCCESS     = '<span style="color:#2e933c;">{}</span>'
        CMD_NAME    = '<span style="color:#8dbf55; font-weight:bold;">{}</span>'

        if cmd == "clear":  # Clear feedback window contents
            self.feedback_txt.clear()
            self.display_feedback()
        elif cmd == "cd":  # Change directories
            try:
                # Change directory
                os.chdir(os.path.abspath(cmd_args))
                self.proc.setWorkingDirectory(os.getcwd())
                # Format command to string as "❯ cmd cmd-args" with colored text
                text = SUCCESS.format(u"\n\u276F ") + CMD_NAME.format(cmd) + " " + cmd_txt.partition(" ")[2]
                self.feedback_txt.append(text)
                # Update prompt representation of path
                self.display_prompt()
            except FileNotFoundError:
                self.feedback_txt.append(WARNING.format(u"\n\u276F ") + f"cd: no such file or directory: {cmd_txt.partition(' ')[2]}")
            finally:
                self.display_feedback()
        elif cmd == "history":  # Display previous commands ran in this terminal session with indexes
            self.feedback_txt.append(SUCCESS.format(u"\n\u276F ") + CMD_NAME.format("history"))
            for index, token in enumerate(self.cmd_history.history):
                self.feedback_txt.append(f"{index:<4}{token}")
            self.display_feedback()
            self.del_cmd()
            return
        elif cmd == "!!":  # Most recent command
            self.del_cmd()
            self.cmd_line.setPlainText(self.cmd_history.history[-1])
            self.move_cursor(qtg.QTextCursor.End)
            return
        elif search(r"[!][0-9]+", cmd):
            index = int(cmd[1:])
            self.del_cmd()
            try:
                print(self.cmd_history.history[index])
                self.cmd_line.insertPlainText(self.cmd_history.history[index])
                self.move_cursor(qtg.QTextCursor.End)
            except IndexError or ValueError:
                self.feedback_txt.append(WARNING.format(u"\n\u276F ") + f"no such event: {index}")
                self.display_feedback()
            finally:
                return
        else:
            if qtc.QStandardPaths.findExecutable(cmd):  # If command is one of the executable commands
                text = SUCCESS.format(u"\n\u276F ") + CMD_NAME.format(cmd) + " " + cmd_txt.partition(" ")[2]
                self.feedback_txt.append(text)
                if self.proc.state() != qtc.QProcess.Running:
                    if "|" in cmd_args or ">" in cmd_args or "<" in cmd_args:
                        self.proc.start(f'sh -c "{cmd} {cmd_args}"')
                    else:
                        self.proc.start(f"{cmd} {cmd_args}")
            else:  # Otherwise command is not executable, display error
                self.feedback_txt.append(WARNING.format(u"\n\u276F ") + f"command not found: {cmd_txt}")
                self.display_feedback()
        # Add command to history
        self.cmd_history.append(cmd_txt.strip())
        # Delete what is currently in the command line text box to get ready for new command
        self.del_cmd()
    
    def read_and_display_cmd_feedback(self):
        """
        Reads feedback from command entered in process and appends it to feedback window.
        """
        try:
            text = str(self.proc.readAll(), encoding = "utf8").strip()
        except TypeError:
            text = str(self.proc.readAll()).strip()
        self.extend_feedback(text)
        
        self.display_feedback()
    
    def extend_feedback(self, text: str):
        """
        Appends some text to the deque of feedback text split on new lines.

        Args:
            text: Text to add to the deque.
        """
        self.feedback_txt.extend([line for line in text.split("\n")])

    def display_feedback(self):
        """
        Display feedback collected in the `self.feedback_txt` deque
        to the feedback widget.
        """
        # Must use HTML format, \n does not work
        self.feedback.setText("<br>".join(self.feedback_txt))
        # Move scroll bar down, else it gets stuck at the top
        self.feedback.verticalScrollBar().setValue(self.feedback.verticalScrollBar().maximum())

    @staticmethod
    def path_reduce(max_len: int, path: str, factor: int) -> str:
        """
        Reduces path to be a certain character length while retaining the most possible information

        Args:
            max_len: Maximum length the path should be
            path: Path to reduce
            factor: Scale to reduce at
        """
        if factor == 0:
            raise Exception("Cannot reduce any further")
        elif len(path) > max_len:
            split_path = path.split("/")
            return TermWidget.path_reduce(max_len, 
                                          "/".join(split_path[i][:factor] for i in range(len(split_path))), 
                                          factor - 1)
        return path

    def display_prompt(self):
        """
        Get current directory path and display it.
        """
        # HTML makes the text highlighted and bold
        path = str(os.getcwd())
        # Find third occurrence of a slash character then splice the string there
        # The purpose of this is to substitute /home/user/foo with ~/foo
        simplified_path = path[path.find("/", path.find("/", path.find("/") + 1) + 1):]
        try:
            # Reduce the path to be under 65 characters
            reduced_path = TermWidget.path_reduce(50, simplified_path, 8)
        except Exception: 
            # If the path could not be reduced, just display the current directory
            reduced_path = simplified_path[simplified_path.rfind("/") + 1:]
        finally:
            self.prompt.setText(f'<span style="background-color:#ff5900; color:#2f2f2f; font-weight:bold">~{reduced_path} </span>')

    def del_cmd(self):
        """
        Deletes all text present in the `cmd_line` textbox.
        """
        self.cmd_line.setPlainText("")
        self.cmd_line.setFocus()
    
    def move_cursor(self, pos, operation: int=0, delete: bool=False):
        """
        Move `cmd_line` cursor to position provided.

        Args:
            pos: Position to move terminal to. Accepts values of the form `qtg.QTextCursor.MoveOpp`. See https://doc.qt.io/qt-6/qtextcursor.html#MoveOperation-enum
            operation: If 0 moves cursor to `pos`. If 1 the cursor selects the text it moves over. (default = 1)
            delete: Delete text selected. operation must be set to 1. (default = false)
        """
        temp_cursor = self.cmd_line.textCursor()
        temp_cursor.movePosition(pos, operation)
        if delete and operation: temp_cursor.removeSelectedText()
        self.cmd_line.setTextCursor(temp_cursor)

    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors)) 
