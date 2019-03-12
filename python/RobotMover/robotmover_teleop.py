
import pygame
import sys
import os
from robotmover_manual import RobotMover



if __name__ == "__main__":
    mover = RobotMover()
    mover.move_forward()   # Bug: First publish doesnt work


    width = 200
    height = 200

    pygame.init()
    main = True

    world = pygame.display.set_mode([width,height])

    backdrop = pygame.image.load(os.path.join(os.path.dirname(os.path.realpath(__file__)),'gui_images','robotgui.png')).convert()
    backdropbox = world.get_rect()
    world.blit(backdrop, backdropbox)

    while main:
        world.blit(backdrop, backdropbox)
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
                main = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT or event.key == ord('a'):
                    print('left')
                    mover.rotate_left()
                if event.key == pygame.K_RIGHT or event.key == ord('d'):
                    print('right')
                    mover.rotate_right()
                if event.key == pygame.K_UP or event.key == ord('w'):
                    print('fwd')
                    mover.move_forward()
                if event.key == pygame.K_DOWN or event.key == ord('s'):
                    print('rev')
                    mover.move_reverse()

            if event.type == pygame.KEYUP:
                print ('stop')
                mover.stop()

                if event.key == ord('q'):
                    pygame.quit()
                    sys.exit(0)
                    main = False
