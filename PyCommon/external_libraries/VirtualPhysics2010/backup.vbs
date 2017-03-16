Set fso = CreateObject("Scripting.FileSystemObject")

If fso.FolderExists("Docs/html") Then
	fso.DeleteFolder("Docs/html")
End if

Sub Cleanup(folder)
	Set files = folder.Files
	For Each file In files
		If fso.GetExtensionName(file.name) = "sdf" Then
			file.Delete
		End if
    Next
    
    Set subfolder = folder.SubFolders 	
    For Each folder in subfolder	
        Cleanup(folder)
        If folder.name = "ipch" or folder.name = "tmp" or folder.name = "Win32" or folder.name = "x64"Then
        	fso.DeleteFolder(folder)
        End if
    Next
End Sub

Cleanup(fso.GetFolder("."))

Sub Compress(Input, ZipFile)
	fso.CreateTextFile(ZipFile, true).WriteLine "PK" & Chr(5) & Chr(6) & String(18, 0)
	Set ZipFile = CreateObject("Shell.Application").NameSpace(ZipFile)
	ZipFile.CopyHere Input
	Do Until ZipFile.items.Count = 1 
		wscript.sleep 100
	Loop
End Sub

today = Replace(Date, "/", "_")
zipfile = "_"&today&".zip"
If fso.FileExists(zipfile) Then
	fso.DeleteFile(zipfile)
End if

pathname = fso.GetAbsolutePathName(".")

Compress pathname, pathname&zipfile

