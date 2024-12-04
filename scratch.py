async def _find_edge(self, eye):
    if eye == 'left':
        eye = self.left_eye
    elif eye == 'right':
        eye = self.right_eye

    while not eye.is_black_or_white():
        await wait(10)

    reflection = await eye.reflection()

    while abs(reflection - await eye.reflection()) < 25:
        await wait(10)

async def _stop_at_edge(self, eye, angle=180, turn_rate=45):
    await multitask(self._turn(angle=angle, turn_rate=turn_rate), self._find_edge(eye), race=True)
    
        

    
        
