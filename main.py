import os
from colorama import Fore, Back, Style
from feylix import Feylix

def run():
    startup()
    prints()
    Feylix().listen()

def startup():
    os.system("color")
    os.system(f"title Feylix")

def prints():
    print(f"""{Fore.CYAN}
             \t\t\t\t    ███████╗███████╗██╗░░░██╗██╗░░░░░██╗██╗░░██╗
             \t\t\t\t    ██╔════╝██╔════╝╚██╗░██╔╝██║░░░░░██║╚██╗██╔╝
             \t\t\t\t    █████╗░░█████╗░░░╚████╔╝░██║░░░░░██║░╚███╔╝░
             \t\t\t\t    ██╔══╝░░██╔══╝░░░░╚██╔╝░░██║░░░░░██║░██╔██╗░
             \t\t\t\t    ██║░░░░░███████╗░░░██║░░░███████╗██║██╔╝╚██╗
             \t\t\t\t    ╚═╝░░░░░╚══════╝░░░╚═╝░░░╚══════╝╚═╝╚═╝░░╚═╝
          {Style.RESET_ALL}""")
    
    print(
    f"{Fore.LIGHTBLACK_EX}[" + f"{Fore.CYAN}x" + f"{Fore.LIGHTBLACK_EX}]{Style.RESET_ALL}"
    " Join my discord server: " +
    f"{Fore.WHITE}discord.gg/KCbjGkBxCW{Style.RESET_ALL}"
    )

run()
