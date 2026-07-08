import os
import sys
import pygame

os.environ.setdefault("SDL_VIDEO_CENTERED", "1")

pygame.init()
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption("Micromouse Simulator")
clock = pygame.time.Clock()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit(0)

    screen.fill((18, 18, 22))
    pygame.draw.rect(screen, (220, 20, 60), pygame.Rect(300, 300, 200, 200))
    pygame.display.flip()
    clock.tick(60)