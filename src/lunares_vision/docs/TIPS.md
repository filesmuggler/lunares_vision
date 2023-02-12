## Runing argparse with arguments as a list

### nargs
nargs='+' takes 1 or more arguments, nargs='*' takes zero or more.
Use like:
```python3
parser.add_argument('-cl','--cam-list', nargs='+', help='<Required> list of camera serial numbers', required=True)
```
```sh
python arg.py -l 1234 2345 3456 4567
```
### append
With append you provide the option multiple times to build up the list.
Use like:
```sh
python arg.py -l 1234 -l 2345 -l 3456 -l 4567
```
## Source
- [Python Docs](https://docs.python.org/3/library/argparse.html)
- [StackOverflow](https://stackoverflow.com/questions/15753701/how-can-i-pass-a-list-as-a-command-line-argument-with-argparse)