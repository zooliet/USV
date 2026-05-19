from customtkinter import CTkToplevel, CTkLabel, CTkFont, CTkEntry, StringVar, CTkButton
from typing import Optional, Tuple, Union


class InputDialog(CTkToplevel):
    def __init__(
        self,
        master,
        title: str = "Input Dialog",
        label_text: str = "Input Dialog",
        entry_text: Optional[str] = None,
    ):
        super().__init__(master)
        self.master = master
        # self.title = title
        self.label_text = label_text
        self.entry_text = entry_text
        self.user_input: Union[str, None] = None

        self.title(title)
        self.lift()  # lift window on top
        self.attributes("-topmost", True)  # stay on top
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after(
            10, self.create_widgets
        )  # create widgets with slight delay, to avoid white flickering of background
        self.resizable(False, False)
        self.grab_set()  # make window modal so that other windows not clickable

    def create_widgets(self):
        self.grid_columnconfigure((0, 1), weight=1)
        # self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(10, weight=1)

        self.label = CTkLabel(self, text=self.label_text, font=CTkFont(size=14))
        self.label.grid(row=0, column=0, columnspan=2, padx=20, pady=20, sticky="new")

        self.entry = CTkEntry(
            self,
            # width=180,
            font=CTkFont(size=14),
            textvariable=StringVar(self, self.entry_text),
        )
        self.entry.grid(row=1, column=0, columnspan=2, padx=40, pady=20, sticky="new")

        self.ok_button = CTkButton(
            master=self,
            # width=50,
            height=40,
            text="Ok",
            command=self.ok_event,
        )
        self.ok_button.grid(
            row=10, column=0, columnspan=1, padx=(40, 40), pady=(20, 40), sticky="s"
        )

        self.cancel_button = CTkButton(
            master=self,
            # width=50,
            height=40,
            text="Cancel",
            command=self.cancel_event,
        )
        self.cancel_button.grid(
            row=10, column=1, columnspan=1, padx=(40, 40), pady=(20, 40), sticky="s"
        )

        # set focus to entry with slight delay, otherwise it won't work
        self.after(150, lambda: self.entry.focus())
        self.entry.bind("<Return>", self.ok_event)

    def ok_event(self):
        self.user_input = self.entry.get()
        self.grab_release()
        self.destroy()

    def cancel_event(self):
        self.grab_release()
        self.destroy()

    def on_closing(self):
        self.grab_release()
        self.destroy()

    def get_input(self):
        self.master.wait_window(self)
        return self.user_input
