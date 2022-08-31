import pygame


class Node:
    def __init__(self, row, col, width, n):
        self.row = row
        self.col = col
        self.x = row * width  # coordinate position on the screen
        self.y = col * width
        self.color = (255, 255, 255)
        self.neighbors = []
        self.width = width
        self.n = n

        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.PURPLE = (128, 0, 128)
        self.ORANGE = (255, 165, 0)
        self.GREY = (128, 128, 128)
        self.TURQUOISE = (64, 224, 208)

    # things are indexed using row columns (row, column)
    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        # have this node already been explored
        return self.color == self.RED

    def is_open(self):
        return self.color == self.GREEN

    def is_barrier(self):
        return self.color == self.BLACK

    def is_start(self):
        return self.color == self.ORANGE

    def is_end(self):
        return self.color == self.TURQUOISE

    def reset(self):
        self.color = self.WHITE

    def make_closed(self):
        # have this node already been explored
        self.color = self.RED

    def make_open(self):
        self.color = self.GREEN

    def make_barrier(self):
        self.color = self.BLACK

    def make_start(self):
        self.color = self.ORANGE

    def make_end(self):
        self.color = self.TURQUOISE

    def make_path(self):
        self.color = self.PURPLE

    def draw(self, win):
        return pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        # checking is the row that we're at is less that the total (rows+1)
        # collision checking; is the row below an obstacle? if not add it to the list of neighbors
        if self.row < self.n - 1 and not grid[self.row + 1][self.col].is_barrier():  # SOUTH
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # NORTH
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # WEST
            self.neighbors.append(grid[self.row][self.col - 1])

        if self.col < self.n - 1 and not grid[self.row][self.col + 1].is_barrier():  # EAST
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.row > 0 and self.col < self.n - 1 and not grid[self.row - 1][self.col + 1].is_barrier():  # NORTHEAST
            self.neighbors.append(grid[self.row - 1][self.col + 1])

        if self.row < self.n - 1 and self.col < self.n - 1 and not grid[self.row + 1][
            self.col + 1].is_barrier():  # SOUTHEAST
            self.neighbors.append(grid[self.row + 1][self.col + 1])

        if self.row < self.n - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():  # SOUTHWEST
            self.neighbors.append(grid[self.row + 1][self.col - 1])

        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():  # NORTHWEST
            self.neighbors.append(grid[self.row - 1][self.col - 1])
