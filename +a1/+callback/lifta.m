function lifta(nlp, src, tar, bounds, varargin)

    nlp.Plant.rigidImpactConstraint(nlp, src, tar, bounds, varargin{:});
    
    removeConstraint(nlp,'tContDomain');
    
end