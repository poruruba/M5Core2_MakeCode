let led2 = 0
custom.setInterval(500, function () {
    custom.led_onoff(led2)
    led2 = (led2 + 1) % 2
})
custom.onButtonPressed(ExButton.C, function () {
    custom.player_step(10, 0)
})
custom.onButtonPressed(ExButton.B, function () {
    custom.player_rotate(ExRotate.RIGHT, 10)
})
custom.onButtonPressed(ExButton.A, function () {
    custom.player_step(-10, 0)
})
custom.onPlayerTouched(function () {
    custom.shock_play(100)
})
