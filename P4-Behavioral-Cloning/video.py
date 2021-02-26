from moviepy.editor import ImageSequenceClip
import argparse
import os

IMAGE_EXT = ['jpeg', 'gif', 'png', 'jpg']


def main():
    parser = argparse.ArgumentParser(description='Create driving video.')
    parser.add_argument(
        'image_folder',
        type=str,
        default='',
        help='Path to image folder. The video will be created from these images.'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=60,
        help='FPS (Frames per second) setting for the video.')
    parser.add_argument(
        '--ext',
        type=str,
        default='mp4',
        help='Video format : mp4 or gif')
    parser.add_argument(
        '--cleanup',
        help='if present, then remove the images folder after video conversion')
    args = parser.parse_args()

    # convert file folder into list firltered for image file types
    image_list = sorted([os.path.join(args.image_folder, image_file)
                        for image_file in os.listdir(args.image_folder)])
    
    image_list = [image_file for image_file in image_list if os.path.splitext(image_file)[1][1:].lower() in IMAGE_EXT]

    print("Creating video {}, FPS={}".format(args.image_folder, args.fps))

    if args.ext == 'mp4':
        clip = ImageSequenceClip(image_list, fps=args.fps)
        clip.write_videofile(args.image_folder + '.mp4')
    else:
        clip = ImageSequenceClip(image_list[::12], fps=args.fps)  # keep gif file small
        clip.write_gif(args.image_folder + '.gif', program='ffmpeg', fps=clip.fps)

    if args.cleanup:
        [os.remove(f'{args.image_folder}/{f}') for f in os.listdir(args.image_folder) if f.endswith('.jpg')]


if __name__ == '__main__':
    main()
