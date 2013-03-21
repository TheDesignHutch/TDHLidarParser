
echo.Collecting LIDARParser...
xcopy /EXCLUDE:collect_bin_excludes.txt /D /C /I /Y .\LIDARParser\*.h ..\LIDARParser_Build
xcopy /D /E /C /I /Y .\LIDARParser\bin\* ..\LIDARParser_Build\bin