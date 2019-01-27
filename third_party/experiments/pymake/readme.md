# Deploy

```bash
# After incrementing the version number
rm dist/*
python setup.py sdist
twine upload dist/*
```