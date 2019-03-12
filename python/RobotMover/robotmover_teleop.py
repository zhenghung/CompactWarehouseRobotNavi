
import pygame
import sys
import os
from robotmover_manual import RobotMover

IMG_DEFAULT = os.path.join(os.path.dirname(os.path.realpath(__file__)),'gui_images','default.png')
IMG_FWD = os.path.join(os.path.dirname(os.path.realpath(__file__)),'gui_images','fwd.png')
IMG_LEFT = os.path.join(os.path.dirname(os.path.realpath(__file__)),'gui_images','left.png')
IMG_RIGHT = os.path.join(os.path.dirname(os.path.realpath(__file__)),'gui_images','right.png')
IMG_REV = os.path.join(os.path.dirname(os.path.realpath(__file__)),'gui_images','rev.png')

if __name__ == "__main__":
    mover = RobotMover()
    mover.move_forward()   # Bug: First publish doesnt work


    width = 500
    height = 500

    pygame.init()
    main = True

    world = pygame.display.set_mode([width,height])

    backdrop = pygame.image.load(IMG_DEFAULT).convert()
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
                    backdrop = pygame.image.load(IMG_LEFT).convert()
                if event.key == pygame.K_RIGHT or event.key == ord('d'):
                    print('right')
                    mover.rotate_right()
                    backdrop = pygame.image.load(IMG_RIGHT).convert()
                if event.key == pygame.K_UP or event.key == ord('w'):
                    print('fwd')
                    mover.move_forward()
                    backdrop = pygame.image.load(IMG_FWD).convert()
                if event.key == pygame.K_DOWN or event.key == ord('s'):
                    print('rev')
                    mover.move_reverse()
                    backdrop = pygame.image.load(IMG_REV).convert()

            if event.type == pygame.KEYUP:
                print ('stop')
                mover.stop()
                backdrop = pygame.image.load(IMG_DEFAULT).convert()

                if event.key == ord('q'):
                    pygame.quit()
                    sys.exit(0)
                    main = False
        
        world.blit(backdrop, backdropbox)
        pygame.display.flip()