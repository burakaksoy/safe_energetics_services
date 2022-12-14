# Robot request service
wt -w 0 nt Powershell -NoExit -c {
    $Host.UI.RawUI.WindowTitle = "Robot Request Service"
    cd ~\safe_energetics_services\src\robot_request_service
    python.exe .\robot_request_service.py
}

Start-Sleep -Seconds 3.0

# IO module service
wt -w 0 nt Powershell -NoExit -c {
    $Host.UI.RawUI.WindowTitle = "IO Service"
    cd ~\safe_energetics_services\src\io_manager_service
    python.exe .\io_manager_service.py
}

Start-Sleep -Seconds 3.0

# Robot request client example
wt -w 0 nt Powershell -NoExit -c {
    $Host.UI.RawUI.WindowTitle = "Robot Request Example Client"
    cd ~\safe_energetics_services\src\robot_request_service
    python.exe .\robot_request_client_interactive.py
}

Start-Sleep -Seconds 1.0

# IO module client example
wt -w 0 nt Powershell -NoExit -c {
    $Host.UI.RawUI.WindowTitle = "IO Example Client"
    cd ~\safe_energetics_services\src\io_manager_service
    python.exe .\io_manager_client_interactive.py
}