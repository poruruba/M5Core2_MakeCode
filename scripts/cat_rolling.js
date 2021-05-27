let x = 0
let y = 0
custom.setInterval(200, function () {
    x = custom.player_position(ExDimension2d.X) - custom.acceleration(ExDimension.X) / 2
    y = custom.player_position(ExDimension2d.Y) + custom.acceleration(ExDimension.Y) / 2
    if (x < 0) {
        x = 0
        custom.shock_play(100)
    }
    if (x >= 320) {
        x = 319
        custom.shock_play(100)
    }
    if (y < 0) {
        y = 0
        custom.shock_play(100)
    }
    if (y >= 240) {
        y = 239
        custom.shock_play(100)
    }
    custom.player_goto(x, y)
})