# Script to remove all emojis from car.ino
$file = "car.ino"
$content = Get-Content $file -Raw -Encoding UTF8

# List of emojis to remove
$emojis = @('🔧','🚗','⬆️','⬇️','⬅️','➡️','📍','🐌','🔄','⚖️','💡','🎮','🎯','✅','❌','📊','🛡️','🔍','⚠️')

foreach ($emoji in $emojis) {
    $content = $content -replace [regex]::Escape($emoji), ''
}

# Also remove any other common emojis that might be missed
$content = $content -replace '[\u{1F600}-\u{1F64F}]', ''  # emoticons
$content = $content -replace '[\u{1F300}-\u{1F5FF}]', ''  # misc symbols
$content = $content -replace '[\u{1F680}-\u{1F6FF}]', ''  # transport
$content = $content -replace '[\u{2600}-\u{26FF}]', ''    # misc symbols
$content = $content -replace '[\u{2700}-\u{27BF}]', ''    # dingbats

Set-Content $file -Value $content -Encoding UTF8
Write-Host "Emojis removed from $file"
